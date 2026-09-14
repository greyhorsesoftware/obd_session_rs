//! Response accumulator — the ONE place bytes from any transport become
//! ELM text (LH0, Link_Handler_Plan).
//!
//! Every transport — the Swift-fed host path (BLE / classic / iOS EA /
//! mock replays via `obd_external_receive_bytes`) and the feature-gated native
//! transports — feeds raw bytes here. Two modes:
//!
//! - **Prompt** (request/response): buffer until the `>` prompt, deliver
//!   everything before the LAST `>` as one response. A byte-exact port of
//!   the Swift `ResponseProcessor` this replaced: raw `\r`-separated text,
//!   trimmed, `SEARCHING…` lines PRESERVED (`detect_addressing`'s no-response
//!   classification needs them), an empty response (stale prompt) is dropped
//!   rather than delivered.
//! - **Line** (monitor / sink): CR/LF-split lines with a trailing-partial
//!   carry, empty lines and bare `>` skipped — the Swift sink branch.
//!
//! Switching modes clears the buffer (Swift `setSinkHandler` did the same).
//! LH5: the native btleplug/RFCOMM transports ride this same byte path
//! (`ByteTransport` → `feed_bytes`); the legacy string feed is gone.

/// What the accumulator produces from a chunk of bytes.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AccEvent {
    /// Prompt mode: one complete response (text before the last `>`).
    Response(String),
    /// Line mode: one monitor line.
    Line(String),
    /// Prompt mode: a prompt arrived with nothing before it (stale `>`).
    /// Delivered so the transport log can show it; nothing completes.
    Dropped(String),
    /// DVI mode (LH6): one complete, checksum-valid binary frame.
    Dvi(crate::link::dvi::codec::DviFrame),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    Prompt,
    Line,
    /// OBDX DVI binary frames (`CMD LEN DATA YY`) — after the `DX DP 1` bootstrap.
    Dvi,
}

/// Accumulates adapter bytes until a complete response (Prompt), line
/// (Line) or binary frame (Dvi).
pub struct ResponseAccumulator {
    buffer: Vec<u8>,
    mode: Mode,
    dvi: crate::link::dvi::codec::FrameParser,
}

impl ResponseAccumulator {
    pub fn new() -> Self {
        Self {
            buffer: Vec::new(),
            mode: Mode::Prompt,
            dvi: crate::link::dvi::codec::FrameParser::new(),
        }
    }

    /// DVI: RX frames carry a 4-byte µs timestamp once `24 02 03 01` took.
    pub fn set_dvi_timestamps(&mut self, on: bool) {
        self.dvi.timestamps = on;
    }

    pub fn mode(&self) -> Mode {
        self.mode
    }

    /// Switch modes. Clears any half-formed buffer (a command buffer must
    /// not leak into the monitor tap, and vice versa).
    pub fn set_mode(&mut self, mode: Mode) {
        self.mode = mode;
        self.buffer.clear();
        self.dvi.reset();
    }

    /// Feed raw transport bytes. Returns every complete event the chunk
    /// produced, in order (a chunk can close several monitor lines).
    pub fn feed_bytes(&mut self, data: &[u8]) -> Vec<AccEvent> {
        if self.mode == Mode::Dvi {
            return self.dvi.feed(data).into_iter().map(AccEvent::Dvi).collect();
        }
        self.buffer.extend_from_slice(data);
        match self.mode {
            Mode::Prompt => self.drain_prompt(),
            Mode::Line => self.drain_lines(),
            Mode::Dvi => unreachable!(),
        }
    }

    fn drain_prompt(&mut self) -> Vec<AccEvent> {
        let Some(prompt) = self.buffer.iter().rposition(|&b| b == b'>') else {
            return Vec::new();
        };
        let text = Self::to_text(&self.buffer[..prompt]);
        // Swift cleared the WHOLE buffer at the prompt — including anything
        // that arrived after it.
        self.buffer.clear();
        let response = text.trim().to_string();
        if response.is_empty() {
            vec![AccEvent::Dropped(
                "Empty response — stale > prompt".to_string(),
            )]
        } else {
            vec![AccEvent::Response(response)]
        }
    }

    fn drain_lines(&mut self) -> Vec<AccEvent> {
        // Split on CR or LF; the tail after the last terminator is the
        // partial carried to the next chunk.
        let Some(last_term) = self.buffer.iter().rposition(|&b| b == b'\r' || b == b'\n') else {
            return Vec::new();
        };
        let complete = self.buffer[..=last_term].to_vec();
        self.buffer.drain(..=last_term);
        let mut events = Vec::new();
        for raw in complete.split(|&b| b == b'\r' || b == b'\n') {
            let line = Self::to_text(raw);
            let trimmed = line.trim_matches(|c| c == ' ' || c == '\t');
            if trimmed.is_empty() || trimmed == ">" {
                continue;
            }
            events.push(AccEvent::Line(trimmed.to_string()));
        }
        events
    }

    /// Adapter output is ASCII. Swift refused to decode a chunk with a
    /// non-ASCII byte (and waited for more, which could never help); we map
    /// such bytes to `?` instead so a corrupt byte cannot wedge a response.
    fn to_text(bytes: &[u8]) -> String {
        bytes
            .iter()
            .map(|&b| if b.is_ascii() { b as char } else { '?' })
            .collect()
    }

    /// Reset the accumulator (on disconnect, timeout, etc.)
    pub fn reset(&mut self) {
        self.buffer.clear();
        self.dvi.reset();
    }

    /// Check if there's partial data buffered.
    pub fn has_pending(&self) -> bool {
        !self.buffer.is_empty()
    }

    /// Force-flush whatever is in the buffer (timeout scenario).
    pub fn flush(&mut self) -> Option<String> {
        if self.buffer.is_empty() {
            return None;
        }
        let raw = Self::to_text(&self.buffer);
        self.buffer.clear();
        Some(raw.trim().to_string())
    }
}

impl Default for ResponseAccumulator {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ---- prompt mode (Swift ResponseProcessor parity) ---------------------

    #[test]
    fn prompt_delivers_text_before_last_prompt() {
        let mut acc = ResponseAccumulator::new();
        let ev = acc.feed_bytes(b"7E8 04 41 0C 1A F8\r\r>");
        assert_eq!(ev, vec![AccEvent::Response("7E8 04 41 0C 1A F8".into())]);
        assert!(!acc.has_pending());
    }

    #[test]
    fn prompt_split_across_ble_notifications() {
        let mut acc = ResponseAccumulator::new();
        assert!(acc.feed_bytes(b"7E8 04 41").is_empty());
        assert!(acc.feed_bytes(b" 0C 1A F8\r\r").is_empty());
        assert_eq!(
            acc.feed_bytes(b">"),
            vec![AccEvent::Response("7E8 04 41 0C 1A F8".into())]
        );
    }

    #[test]
    fn prompt_preserves_searching_lines() {
        // detect_addressing skips SEARCHING itself and classifies an
        // all-SEARCHING reply as "no response" — it must SEE the line.
        let mut acc = ResponseAccumulator::new();
        let ev = acc.feed_bytes(b"SEARCHING...\r7E8 06 41 00 BE 3F A8 13\r\r>");
        assert_eq!(
            ev,
            vec![AccEvent::Response(
                "SEARCHING...\r7E8 06 41 00 BE 3F A8 13".into()
            )]
        );
        let ev = acc.feed_bytes(b"SEARCHING...\rUNABLE TO CONNECT\r\r>");
        assert_eq!(
            ev,
            vec![AccEvent::Response("SEARCHING...\rUNABLE TO CONNECT".into())]
        );
    }

    #[test]
    fn prompt_stale_prompt_is_dropped_not_delivered() {
        let mut acc = ResponseAccumulator::new();
        let ev = acc.feed_bytes(b"\r\r>");
        assert!(matches!(ev.as_slice(), [AccEvent::Dropped(_)]));
        let ev = acc.feed_bytes(b">");
        assert!(matches!(ev.as_slice(), [AccEvent::Dropped(_)]));
    }

    #[test]
    fn prompt_last_prompt_wins_and_buffer_fully_cleared() {
        // Two prompts in one chunk: Swift took everything before the LAST
        // one as a single response and cleared the buffer.
        let mut acc = ResponseAccumulator::new();
        let ev = acc.feed_bytes(b"OK\r\r>OK\r\r>");
        assert_eq!(ev, vec![AccEvent::Response("OK\r\r>OK".into())]);
        assert!(!acc.has_pending());
    }

    #[test]
    fn prompt_non_ascii_byte_does_not_wedge() {
        let mut acc = ResponseAccumulator::new();
        let ev = acc.feed_bytes(b"NO \xffDATA\r>");
        assert_eq!(ev, vec![AccEvent::Response("NO ?DATA".into())]);
    }

    // ---- line mode (Swift sink branch parity) -----------------------------

    #[test]
    fn line_mode_splits_and_carries_partial() {
        let mut acc = ResponseAccumulator::new();
        acc.set_mode(Mode::Line);
        let ev = acc.feed_bytes(b"7E8 03 41 0C 1A F8\r6A0 04 12");
        assert_eq!(ev, vec![AccEvent::Line("7E8 03 41 0C 1A F8".into())]);
        assert!(acc.has_pending());
        let ev = acc.feed_bytes(b(" 34 56\r\n"));
        assert_eq!(ev, vec![AccEvent::Line("6A0 04 12 34 56".into())]);
        assert!(!acc.has_pending());
    }

    #[test]
    fn line_mode_skips_blank_and_bare_prompt_but_keeps_glued_prompt() {
        let mut acc = ResponseAccumulator::new();
        acc.set_mode(Mode::Line);
        let ev = acc.feed_bytes(b"\r\r>\r>6A0 04 11 22 33 44\rSTOPPED\r");
        assert_eq!(
            ev,
            vec![
                AccEvent::Line(">6A0 04 11 22 33 44".into()),
                AccEvent::Line("STOPPED".into()),
            ]
        );
    }

    #[test]
    fn mode_switch_clears_half_formed_buffer() {
        let mut acc = ResponseAccumulator::new();
        acc.feed_bytes(b"7E8 04 41");
        acc.set_mode(Mode::Line);
        assert!(!acc.has_pending());
        acc.feed_bytes(b"partial");
        acc.set_mode(Mode::Prompt);
        assert!(!acc.has_pending());
        assert_eq!(
            acc.feed_bytes(b"OK\r>"),
            vec![AccEvent::Response("OK".into())]
        );
    }

    fn b(s: &str) -> &[u8] {
        s.as_bytes()
    }

    // ---- legacy native API ------------------------------------------------

    #[test]
    fn test_flush_and_reset() {
        let mut acc = ResponseAccumulator::new();
        acc.feed_bytes(b"7E8 04 41 0C");
        assert!(acc.has_pending());
        assert_eq!(acc.flush().unwrap(), "7E8 04 41 0C");
        assert!(!acc.has_pending());
        acc.feed_bytes(b"7E8 04");
        acc.reset();
        assert!(!acc.has_pending());
    }
}
