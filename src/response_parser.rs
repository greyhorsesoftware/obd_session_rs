//! Core OBD response parser
//!
//! Parses raw hex response strings from the ELM327/OBD adapter into structured data.
//! Handles controller ID extraction, echo byte stripping, multi-controller responses,
//! and ISO-TP multi-frame assembly.

use serde::Serialize;
use std::collections::HashMap;

use crate::addressing::{Addressing, Bus};
use crate::link::Payload;

/// A single controller's parsed response
#[derive(Debug, Clone, Serialize)]
pub struct ControllerResponse {
    /// Raw hex string for this controller's portion
    pub raw_hex: String,
    /// Extracted data bytes after stripping echo bytes
    pub data_bytes: Vec<u8>,
    /// Controller ID (e.g., "00") or "DEFAULT" if no prefix
    pub controller_id: String,
    /// Whether this response contains valid data
    pub is_valid: bool,
}

/// Parsed response type discriminator
#[derive(Debug, Clone, Serialize, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum ParseType {
    PidResponse,
    TroubleCodes,
    MonitorStatus,
    PidSupport,
    Mode06,
}

/// Type-specific parsed data
#[derive(Debug, Clone, Serialize)]
#[serde(untagged)]
pub enum ParseTypeData {
    /// Generic PID response — data_bytes are on each ControllerResponse
    PidResponse,
    /// Placeholder for specialized parsers (Phase 2)
    TroubleCodes {
        codes: Vec<String>,
    },
    MonitorStatus {
        raw_bytes: Vec<u8>,
    },
    PidSupport {
        supported_pids: Vec<String>,
    },
    Mode06 {
        raw_bytes: Vec<u8>,
    },
}

/// Full parsed response with all controllers
#[derive(Debug, Clone, Serialize)]
pub struct ParsedResponse {
    /// What kind of response this is
    #[serde(rename = "type")]
    pub parse_type: ParseType,
    /// All controller responses keyed by controller ID
    pub all_controllers: HashMap<String, ControllerResponse>,
}

/// Default controller ID when no prefix is present
const DEFAULT_CONTROLLER: &str = "DEFAULT";

/// Check if data is a non-parseable response (error or status from adapter).
/// LH1: one table — `link::classify_line`. `SEARCHING…` is NOT non-data here:
/// it is noise `sanitize_response` strips, and the data that follows a
/// re-search must still parse (the pre-LH1 table never listed it; LH2's
/// review caught the regression via the FS2 probes).
pub(crate) fn is_non_data_response(data: &str) -> bool {
    let t = data.trim();
    if t.starts_with("SEARCHING") {
        return false;
    }
    crate::link::classify_line(t).is_some()
}

/// Sanitize raw adapter response by removing ELM327 noise:
/// - Command echo (e.g., "0120\r" at the start)
/// - "SEARCHING..." status messages
/// - Blank lines
fn sanitize_response(data: &str, command: &str) -> String {
    // Normalize \r to \n so bare-CR adapters (ELM327) split correctly
    let normalized = data.replace('\r', "\n");
    normalized
        .lines()
        .map(|line| line.trim())
        .filter(|line| {
            !line.is_empty()
                && !line.starts_with("SEARCHING")
                && !line.eq_ignore_ascii_case(command)
        })
        .collect::<Vec<_>>()
        .join("\n")
}

/// Check if command is an AT command
fn is_at_command(command: &str) -> bool {
    let upper = command.trim().to_uppercase();
    upper.starts_with("AT") || upper.starts_with("ST")
}

/// Extract mode and PID from a command string
/// e.g., "010C" -> ("01", "0C"), "03" -> ("03", ""), "0100" -> ("01", "00")
/// e.g., "22F40C" -> ("22", "F40C")
fn parse_command(command: &str) -> (String, String) {
    let cmd = command.trim().to_uppercase();
    if cmd.len() < 2 {
        return (cmd, String::new());
    }
    let mode = &cmd[..2];
    let pid = &cmd[2..];
    (mode.to_string(), pid.to_string())
}

/// Calculate expected echo bytes for a given mode and PID
/// Mode 01 PID 0C -> echo "41 0C" (2 bytes)
/// Mode 03 -> echo "43" (1 byte)
/// Mode 06 PID support -> echo "46 XX" (2 bytes)
/// Mode 06 test data  -> echo "46" only (1 byte) — MID repeats in each 9-byte block
/// Mode 09 PID 02 -> echo "49 02" (2 bytes)
/// Mode 22 PID F40C -> echo "62 F4 0C" (3 bytes)
fn echo_byte_count(mode: &str, pid: &str) -> usize {
    match mode {
        "03" | "07" | "0A" => 1, // just response mode byte (43, 47, 4A)
        "04" => 1,               // 44
        "06" => {
            // PID support queries (00/20/40/60) echo "46 XX"
            // Test data queries echo only "46" — MID is in each result block
            match pid {
                "00" | "20" | "40" | "60" => 2,
                _ => 1,
            }
        }
        "22" => {
            // Mode 22: 62 + 2-byte PID
            if pid.len() == 4 {
                3
            } else {
                1
            }
        }
        _ => {
            // Modes 01, 02, 05, 06, 09, etc.: response mode byte + PID byte(s)
            if pid.is_empty() {
                1
            } else {
                1 + (pid.len() + 1) / 2
            }
        }
    }
}

/// Detect and assemble ISO-TP multi-frame responses
/// Input: lines like ["7E8 10 0B 41 78 0D 0A ED 09", "7E8 21 1A F8 00 00 ..."]
/// Returns: (controller_id, assembled hex tokens) or None if not multi-frame
fn assemble_iso_tp(lines: &[&str], bus: Bus) -> Option<Vec<(String, Vec<String>)>> {
    if lines.len() < 2 {
        return None;
    }

    // How many leading tokens are the controller header (0 if headerless), via the Bus.
    let header_len = |tokens: &[&str]| bus.read(tokens).map(|(_, n)| n).unwrap_or(0);

    // Group lines by controller ID
    let mut controller_lines: HashMap<String, Vec<&str>> = HashMap::new();
    for line in lines {
        let tokens: Vec<&str> = line.split_whitespace().collect();
        if tokens.is_empty() {
            continue;
        }
        let ctrl = bus
            .read(&tokens)
            .map(|(c, _)| c.key())
            .unwrap_or_else(|| DEFAULT_CONTROLLER.to_string());
        controller_lines.entry(ctrl).or_default().push(line);
    }

    // Single-frame pass-through shared by the "one line" and "not ISO-TP" arms.
    let single_frame = |first_tokens: &[&str], data_start: usize| -> Vec<String> {
        let mut tokens: Vec<String> = first_tokens[data_start..]
            .iter()
            .map(|t| t.to_uppercase())
            .collect();
        strip_pci_byte(&mut tokens);
        tokens
    };

    let mut results = Vec::new();
    let mut single_frame_controllers = Vec::new();
    let mut found_multiframe = false;

    for (ctrl_id, ctrl_lines) in &controller_lines {
        let first_tokens: Vec<&str> = ctrl_lines[0].split_whitespace().collect();
        let data_start = header_len(&first_tokens);

        if ctrl_lines.len() < 2 || data_start >= first_tokens.len() {
            // Single line for this controller — pass through as single-frame
            // (will be included if any other controller is multi-frame)
            single_frame_controllers
                .push((ctrl_id.clone(), single_frame(&first_tokens, data_start)));
            continue;
        }

        let first_byte = first_tokens[data_start];
        // Check for ISO-TP first frame (upper nibble = 1)
        let is_first_frame = if first_byte.len() == 2 {
            u8::from_str_radix(first_byte, 16)
                .map(|b| (b >> 4) == 1)
                .unwrap_or(false)
        } else {
            false
        };

        if !is_first_frame {
            // Not ISO-TP — treat as single-frame
            single_frame_controllers
                .push((ctrl_id.clone(), single_frame(&first_tokens, data_start)));
            continue;
        }

        found_multiframe = true;

        // First frame: skip controller ID, extract total data length, then data
        // Format: [ctrl] 1X LL data... where X:LL is the 12-bit total data length
        let pci_byte = u8::from_str_radix(first_tokens[data_start], 16).unwrap_or(0);
        let len_byte =
            u8::from_str_radix(first_tokens.get(data_start + 1).unwrap_or(&"0"), 16).unwrap_or(0);
        let total_data_len = (((pci_byte & 0x0F) as usize) << 8) | (len_byte as usize);

        let first_frame_data: Vec<String> = first_tokens[data_start + 2..]
            .iter()
            .map(|t| t.to_uppercase())
            .collect();

        let mut assembled = first_frame_data;

        // Consecutive frames: [ctrl] 2X data...
        for line in &ctrl_lines[1..] {
            let tokens: Vec<&str> = line.split_whitespace().collect();
            let d_start = header_len(&tokens);
            if d_start >= tokens.len() {
                continue;
            }
            // Skip the sequence byte (21, 22, 23, etc.)
            let seq_byte = tokens[d_start];
            if let Ok(b) = u8::from_str_radix(seq_byte, 16) {
                if (b >> 4) != 2 {
                    continue; // Not a consecutive frame
                }
            }
            for token in &tokens[d_start + 1..] {
                assembled.push(token.to_uppercase());
            }
        }

        // Truncate to declared ISO-TP data length
        if total_data_len > 0 && assembled.len() > total_data_len {
            assembled.truncate(total_data_len);
        }
        // LH7 (review): an assembly SHORT of its declared total (a lost CF)
        // is not a payload — the member gets no data (stays due) rather
        // than decoding garbage or teaching a short length.
        if total_data_len > 0 && assembled.len() < total_data_len {
            continue;
        }

        results.push((ctrl_id.clone(), assembled));
    }

    if found_multiframe {
        // Include single-frame controllers alongside multi-frame ones
        results.extend(single_frame_controllers);
        Some(results)
    } else {
        None
    }
}

/// Strip the CAN single-frame PCI (length) byte if present.
/// In CAN frames, the first byte after the controller ID has upper nibble = 0
/// and encodes the data length (e.g., "03" means 3 payload bytes follow).
fn strip_pci_byte(tokens: &mut Vec<String>) {
    if tokens.is_empty() {
        return;
    }
    if let Ok(b) = u8::from_str_radix(&tokens[0], 16) {
        if (b >> 4) == 0 && b > 0 {
            tokens.remove(0);
            // Bound the payload to the PCI-declared length. The remaining
            // CAN bytes are ECU pad and NOT always 00 — the Gladiator pads
            // with stale buffer bytes (bench 2026-08-02: `06 41 0C 0C 68 0D
            // 00 56` — the 0x56 tail walked into the chunk splitter as an
            // unknown member echo and killed the whole frame, freezing RPM
            // under STN periodic; GM's 00 pads masked the gap).
            let n = b as usize;
            // Only truncate GENUINE single frames (≤7 data bytes follow the
            // PCI — the CAN frame's physical limit). Longer token runs are
            // flattened/assembled payloads (special long pids, test formats)
            // where the leading byte merely resembles a PCI and the tail is
            // real data, not pad (SpecialPidTests 0170/0183/018B/01A1 broke
            // on unconditional truncation).
            if tokens.len() > n && tokens.len() <= 7 {
                tokens.truncate(n);
            }
        }
    }
}

/// Split a single-line multi-controller response
/// e.g., "7E8 04 41 0C 1A F8 7E9 04 41 0C 20 40"
/// Returns list of (controller_id, tokens) with CAN length bytes stripped.
/// `bus.read` recognizes a header (1 token on 11-bit, 4 on 29-bit) at each position; the key
/// is the uniform `Controller::key()` ("7E0", "10").
fn split_controllers(tokens: &[&str], bus: Bus) -> Vec<(String, Vec<String>)> {
    let mut results = Vec::new();
    let mut current_ctrl = DEFAULT_CONTROLLER.to_string();
    let mut current_tokens: Vec<String> = Vec::new();

    let mut i = 0;
    while i < tokens.len() {
        if let Some((ctrl, consumed)) = bus.read(&tokens[i..]) {
            if !current_tokens.is_empty() {
                strip_pci_byte(&mut current_tokens);
                results.push((current_ctrl.clone(), current_tokens.clone()));
                current_tokens.clear();
            }
            current_ctrl = ctrl.key();
            i += consumed;
        } else {
            current_tokens.push(tokens[i].to_uppercase());
            i += 1;
        }
    }
    if !current_tokens.is_empty() {
        strip_pci_byte(&mut current_tokens);
        results.push((current_ctrl, current_tokens));
    }

    results
}

/// Determine the parse type from the command
pub fn detect_parse_type(command: &str) -> Option<ParseType> {
    if is_at_command(command) {
        return None;
    }
    let (mode, pid) = parse_command(command);
    match mode.as_str() {
        "03" | "07" | "0A" => Some(ParseType::TroubleCodes),
        "01" if pid == "01" => Some(ParseType::MonitorStatus),
        "01" if matches!(pid.as_str(), "00" | "20" | "40" | "60") => Some(ParseType::PidSupport),
        "06" if matches!(pid.as_str(), "00" | "20" | "40" | "60") => Some(ParseType::PidSupport),
        "09" if pid == "00" => Some(ParseType::PidSupport),
        "06" => Some(ParseType::Mode06),
        _ => Some(ParseType::PidResponse),
    }
}

/// Main entry point: parse a raw response string given the command that produced it.
///
/// Returns None for AT commands, errors, and non-data responses.
/// Returns Some(ParsedResponse) with structured controller data otherwise.
pub fn parse_response(command: &str, data: &str) -> Option<ParsedResponse> {
    // Default to 11-bit CAN — behavior-preserving for existing callers. The session passes the
    // detected addressing via `parse_response_with_bus` once C2/C3 wire it up.
    parse_response_with_bus(command, data, Bus::new(Addressing::Can11))
}

/// Like [`parse_response`], but detects the addressing from the response text itself (via
/// [`Addressing::sniff`] on the first plausible line): `7E8 …` parses as 11-bit, `18 DA F1 xx …`
/// as 29-bit, and anything unrecognizable falls back to 11-bit (preserving headerless behavior).
/// For session-less callers — e.g. the PID editor's test-data preview, where the pasted sample
/// may come from either an 11-bit or a 29-bit adapter log.
pub fn parse_response_sniffed(command: &str, data: &str) -> Option<ParsedResponse> {
    let sniffed = data
        .replace('\r', "\n")
        .lines()
        .map(str::trim)
        .filter(|l| !l.is_empty() && !l.starts_with("SEARCHING"))
        .find_map(|line| {
            let tokens: Vec<&str> = line.split_whitespace().collect();
            Addressing::sniff(&tokens)
        })
        .unwrap_or(Addressing::Can11);
    parse_response_with_bus(command, data, Bus::new(sniffed))
}

/// Shared framing preamble: split sanitized response text into per-controller
/// token lists — ISO-TP reassembly when a multi-frame is present, else the
/// single-line controller split.
fn frame_controllers(clean: &str, bus: Bus) -> Vec<(String, Vec<String>)> {
    let lines: Vec<&str> = clean.lines().collect();
    if let Some(assembled) = assemble_iso_tp(&lines, bus) {
        assembled
    } else {
        let all_tokens: Vec<&str> = clean.split_whitespace().collect();
        split_controllers(&all_tokens, bus)
    }
}

/// Like [`parse_response`], but with the session's negotiated [`Bus`] so 29-bit headers are
/// recognized. Controller header recognition and framing are the only protocol-specific steps;
/// everything downstream (echo strip, decoders) is header-independent.
pub fn parse_response_with_bus(command: &str, data: &str, bus: Bus) -> Option<ParsedResponse> {
    if is_at_command(command) {
        return None;
    }
    from_payloads(command, &text_payloads(command, data, bus))
}

/// LH7: the text dialect's framing — ELM noise stripped (command echo,
/// `SEARCHING…`, blanks), ISO-TP assembled, one payload per responding
/// controller with the single-frame PCI dropped. `command` is the wire string
/// (its echo line is what gets dropped). Empty when the text is a status
/// token (`NO DATA`, `OK`, `CAN ERROR`…) or carries no frame line. Tokens
/// that aren't hex are skipped (as the parse always did).
pub fn text_payloads(command: &str, data: &str, bus: Bus) -> Vec<Payload> {
    text_frames(command, data, bus)
        .map(|framed| {
            framed
                .into_iter()
                .map(|(controller, tokens)| Payload {
                    controller,
                    bytes: tokens
                        .iter()
                        .filter_map(|t| u8::from_str_radix(t, 16).ok())
                        .collect(),
                })
                .collect()
        })
        .unwrap_or_default()
}

/// The text preamble shared by `text_payloads` and the chunk splitter's
/// text wrapper: `None` = a status token / nothing to frame.
fn text_frames(command: &str, data: &str, bus: Bus) -> Option<Vec<(String, Vec<String>)>> {
    if is_non_data_response(data) {
        return None;
    }
    let clean = sanitize_response(data, command);
    if clean.is_empty() || is_non_data_response(&clean) {
        return None;
    }
    let framed = frame_controllers(&clean, bus);
    if framed.is_empty() {
        None
    } else {
        Some(framed)
    }
}

/// LH7: THE constructor of `ParsedResponse` — the parse for `command` over
/// typed payloads (the text path arrives here through `text_payloads`; DVI
/// directly). The echo (`41 0C` / `62 F4 0C`) the command implies is
/// stripped per controller; `raw_hex` is the payload rendered.
pub fn from_payloads(command: &str, payloads: &[Payload]) -> Option<ParsedResponse> {
    if is_at_command(command) || payloads.is_empty() {
        return None;
    }
    let parse_type = detect_parse_type(command)?;
    let (mode, pid) = parse_command(command);
    let skip = echo_byte_count(&mode, &pid);

    let mut all_controllers = HashMap::new();
    for p in payloads {
        let data_bytes: Vec<u8> = p.bytes.iter().skip(skip).copied().collect();
        let is_valid = !data_bytes.is_empty();
        all_controllers.insert(
            p.controller.clone(),
            ControllerResponse {
                raw_hex: hex_join(&p.bytes),
                data_bytes,
                controller_id: p.controller.clone(),
                is_valid,
            },
        );
    }
    Some(ParsedResponse {
        parse_type,
        all_controllers,
    })
}

/// `41 0C 1A F8` rendering of payload bytes.
pub fn hex_join(bytes: &[u8]) -> String {
    bytes
        .iter()
        .map(|b| format!("{b:02X}"))
        .collect::<Vec<_>>()
        .join(" ")
}

/// B1 splitter failure — the caller (session demux) maps `Validation` to
/// demotion: members stay due, their learned lengths are invalidated, and the
/// standing admission rule ("only length-confirmed members join a chunk")
/// routes them solo to be re-observed. `NonData` is an ordinary empty/error
/// response (NO DATA etc.) — no invalidation, members simply stay due.
#[derive(Debug, Clone, PartialEq)]
pub enum ChunkSplitError {
    NonData,
    Validation(String),
}

/// B1/B3: echo-driven splitter for multi-PID responses — Mode 01 chunks
/// (`41` reply, 1-byte PID echoes) and Mode 22 multi-DID (`62` reply,
/// 2-byte DID echoes; Mustang golden: `62 F4 0C <2B> F4 05 <1B>`). The mode
/// comes from the members themselves (the builder never mixes modes in one
/// segment).
///
/// `members` are the chunk's request PIDs with their LEARNED data lengths
/// (`("010C", 2)` / `("22F40C", 2)`), in request order. The response payload
/// per controller is `<reply> <echo> <data×len> <echo> <data×len> … [00
/// padding]` — the scan reads an echo, looks up the learned length, slices,
/// repeats. Controllers may answer any SUBSET in any order (per-ECU subsets
/// are normal); a member no controller echoed is absent from the result
/// (silent omission — it stays due). Scanning stops at a 0x00 lead byte that
/// isn't a member echo (CF padding). Unknown echo, duplicate echo, truncated
/// data, or a non-reply payload = `Validation` — a mis-slice means a learned
/// length is stale, so the whole chunk demotes rather than risk attributing
/// bytes to the wrong PID.
///
/// Returns pid → ParsedResponse shaped exactly like [`parse_response_with_bus`]
/// output for a solo request of that pid — downstream (controller selection,
/// decode, Swift) can't tell a sliced member from a solo response.
pub fn split_multi_pid_response(
    members: &[(String, u8)],
    data: &str,
    bus: Bus,
) -> Result<HashMap<String, ParsedResponse>, ChunkSplitError> {
    let (mode, _, _) = chunk_mode(members)?;
    let wire: String = format!(
        "{mode}{}",
        members
            .iter()
            .map(|(p, _)| p.strip_prefix(mode).unwrap_or(p))
            .collect::<String>()
    );
    let framed = text_frames(&wire, data, bus).ok_or(ChunkSplitError::NonData)?;
    let mut payloads: Vec<Payload> = Vec::with_capacity(framed.len());
    for (controller, tokens) in framed {
        let bytes: Vec<u8> = tokens
            .iter()
            .map(|t| u8::from_str_radix(t, 16))
            .collect::<Result<_, _>>()
            .map_err(|_| ChunkSplitError::Validation(format!("non-hex token from {controller}")))?;
        payloads.push(Payload { controller, bytes });
    }
    split_multi_pid(members, &payloads)
}

/// The chunk's mode → (mode, reply byte, echo width). The builder never
/// mixes modes in one segment.
fn chunk_mode(members: &[(String, u8)]) -> Result<(&'static str, u8, usize), ChunkSplitError> {
    let Some((mode, _)) = members.first().map(|(p, _)| p.split_at(2)) else {
        return Err(ChunkSplitError::Validation("no members".to_string()));
    };
    match mode {
        "01" => Ok(("01", 0x41u8, 1usize)),
        "22" => Ok(("22", 0x62u8, 2usize)),
        other => Err(ChunkSplitError::Validation(format!(
            "unchunkable mode {other}"
        ))),
    }
}

/// LH7: the echo-driven splitter over typed payloads — what the engine's
/// demux calls for every chunk segment (ELM and DVI alike). Semantics as
/// documented on `split_multi_pid_response`.
pub fn split_multi_pid(
    members: &[(String, u8)],
    payloads: &[Payload],
) -> Result<HashMap<String, ParsedResponse>, ChunkSplitError> {
    let (mode, reply_byte, echo_width) = chunk_mode(members)?;
    if payloads.is_empty() {
        return Err(ChunkSplitError::NonData);
    }

    // echo value → (full pid, learned length)
    let mut by_echo: HashMap<u16, (String, u8)> = HashMap::new();
    for (pid, len) in members {
        if !pid.to_uppercase().starts_with(mode) {
            return Err(ChunkSplitError::Validation(format!(
                "mixed-mode member {pid}"
            )));
        }
        let suffix = pid.strip_prefix(mode).unwrap_or(pid);
        let Ok(echo) = u16::from_str_radix(suffix, 16) else {
            return Err(ChunkSplitError::Validation(format!("bad member pid {pid}")));
        };
        by_echo.insert(echo, (pid.to_uppercase(), *len));
    }

    let mut out: HashMap<String, ParsedResponse> = HashMap::new();
    for p in payloads {
        let ctrl_id = &p.controller;
        let bytes = &p.bytes;
        if bytes.first() != Some(&reply_byte) {
            return Err(ChunkSplitError::Validation(format!(
                "controller {ctrl_id} payload is not a {reply_byte:02X} response"
            )));
        }
        let mut i = 1;
        let mut seen: Vec<u16> = Vec::new();
        while i < bytes.len() {
            // CF padding after the last member (Mode 22 members with a 00
            // high byte are excluded from chunking for exactly this reason).
            if bytes[i] == 0x00 && (echo_width == 2 || by_echo.get(&(bytes[i] as u16)).is_none()) {
                break;
            }
            if i + echo_width > bytes.len() {
                return Err(ChunkSplitError::Validation(format!(
                    "truncated echo from {ctrl_id}"
                )));
            }
            let echo: u16 = if echo_width == 1 {
                bytes[i] as u16
            } else {
                ((bytes[i] as u16) << 8) | bytes[i + 1] as u16
            };
            match by_echo.get(&echo) {
                Some((pid, len)) => {
                    if seen.contains(&echo) {
                        return Err(ChunkSplitError::Validation(format!(
                            "duplicate echo {echo:02X} from {ctrl_id}"
                        )));
                    }
                    let end = i + echo_width + *len as usize;
                    if end > bytes.len() {
                        return Err(ChunkSplitError::Validation(format!(
                            "truncated data for {pid} from {ctrl_id}"
                        )));
                    }
                    seen.push(echo);
                    let data_bytes = bytes[i + echo_width..end].to_vec();
                    let echo_bytes: Vec<u8> = if echo_width == 1 {
                        vec![echo as u8]
                    } else {
                        vec![(echo >> 8) as u8, echo as u8]
                    };
                    let raw_hex = hex_join(
                        &std::iter::once(reply_byte)
                            .chain(echo_bytes.into_iter())
                            .chain(data_bytes.iter().copied())
                            .collect::<Vec<u8>>(),
                    );
                    out.entry(pid.clone())
                        .or_insert_with(|| ParsedResponse {
                            parse_type: ParseType::PidResponse,
                            all_controllers: HashMap::new(),
                        })
                        .all_controllers
                        .insert(
                            ctrl_id.clone(),
                            ControllerResponse {
                                raw_hex,
                                data_bytes,
                                controller_id: ctrl_id.clone(),
                                is_valid: true,
                            },
                        );
                    i = end;
                }
                None if echo == 0x00 => break, // CF padding after the last member
                None => {
                    return Err(ChunkSplitError::Validation(format!(
                        "unknown echo {echo:02X} from {ctrl_id}"
                    )));
                }
            }
        }
    }
    Ok(out)
}

/// B2: does a `010C0D` vehicle-probe response prove multi-PID support?
/// Capable iff BOTH pids come back (any controller). Lengths are the
/// J1979-standard 2/1 — no learned state needed at probe time. One pid,
/// NRC, NO DATA, garbage → not capable (stay serial).
pub fn chunk_probe_capable(payloads: &[Payload]) -> bool {
    split_multi_pid(
        &[("010C".to_string(), 2), ("010D".to_string(), 1)],
        payloads,
    )
    .map(|m| m.contains_key("010C") && m.contains_key("010D"))
    .unwrap_or(false)
}

/// Select a controller from a ParsedResponse
///
/// Tries `preferred` first, then walks `priority` list, then picks the first valid one.
pub fn select_controller<'a>(
    response: &'a ParsedResponse,
    preferred: Option<&str>,
    priority: &[&str],
) -> Option<&'a ControllerResponse> {
    // Try preferred controller
    if let Some(pref) = preferred {
        if let Some(ctrl) = response.all_controllers.get(pref) {
            if ctrl.is_valid {
                return Some(ctrl);
            }
        }
    }

    // Try priority list
    for p in priority {
        if let Some(ctrl) = response.all_controllers.get(*p) {
            if ctrl.is_valid {
                return Some(ctrl);
            }
        }
    }

    // Deterministic fallback: lowest ECU key first (decision D/H). `all_controllers` is a
    // HashMap (nondeterministic iteration), so "first valid" would be run-to-run unstable. Keys
    // are hex request ids ("7E0" < "7E1"; 29-bit "10" < "28"), so string order == ECU order
    // within a session's protocol — lowest is the powertrain ECU, usually the one wanted.
    // (GB14 note: low-range keys like "254" sort below "7E0", but body modules never answer
    // the functional requests this fallback serves, so the mix can't arise.)
    response
        .all_controllers
        .values()
        .filter(|c| c.is_valid)
        .min_by(|a, b| a.controller_id.cmp(&b.controller_id))
}

#[cfg(test)]
mod tests {
    use super::*;

    // -------------------------------------------------------
    // Helper: load mock_data/default.json
    // -------------------------------------------------------
    fn load_mock_data() -> serde_json::Value {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/mock_data/default.json");
        let contents = std::fs::read_to_string(path).expect("failed to read mock data");
        serde_json::from_str(&contents).expect("failed to parse mock data")
    }

    /// Join multi-line mock responses (array of strings) into newline-separated string
    fn mock_lines(arr: &serde_json::Value) -> String {
        arr.as_array()
            .unwrap()
            .iter()
            .map(|v| v.as_str().unwrap())
            .collect::<Vec<_>>()
            .join("\n")
    }

    // -------------------------------------------------------
    // C1 Commit 1 guard: the 11-bit refactor is behavior-preserving on the wire.
    // The controller *key* is the request id ("7E8" → "7E0", GB14), but the ATSH we send and the parsed
    // data bytes must be byte-identical to before — proving the rename is safe noise.
    // -------------------------------------------------------

    use crate::addressing::Controller;

    // -------------------------------------------------------
    // C1 Commit 2: 29-bit parsing, driven by the captured Jeep/Tahoe frames.
    // -------------------------------------------------------

    fn ascii(bytes: &[u8]) -> String {
        bytes
            .iter()
            .filter(|&&b| (0x20..0x7F).contains(&b))
            .map(|&b| b as char)
            .collect()
    }

    #[test]
    fn parse_29bit_two_ecu_support_bitmap_jeep() {
        let bus = Bus::new(Addressing::Can29 { tester: 0xF1 });
        let data = "18 DA F1 10 06 41 00 BF FE B9 93\n18 DA F1 18 06 41 00 98 18 00 01";
        let parsed = parse_response_with_bus("0100", data, bus).unwrap();
        // Split into the two Jeep ECUs by source byte, keyed uniformly.
        assert_eq!(parsed.all_controllers.len(), 2);
        assert_eq!(
            parsed.all_controllers["10"].data_bytes,
            vec![0xBF, 0xFE, 0xB9, 0x93]
        );
        assert_eq!(
            parsed.all_controllers["18"].data_bytes,
            vec![0x98, 0x18, 0x00, 0x01]
        );
    }

    #[test]
    fn parse_29bit_multiframe_vin_jeep() {
        let bus = Bus::new(Addressing::Can29 { tester: 0xF1 });
        let data = "18 DA F1 10 10 14 49 02 01 31 43 34\n\
                    18 DA F1 10 21 4D 4F 43 4B 32 39 53\n\
                    18 DA F1 10 22 42 49 54 30 30 30 31";
        let parsed = parse_response_with_bus("0902", data, bus).unwrap();
        let ctrl = &parsed.all_controllers["10"];
        // After echo strip (49 02) the payload is [count=01, VIN ascii…]; skip count → VIN.
        assert_eq!(ascii(&ctrl.data_bytes[1..]), "1C4MOCK29SBIT0001");
    }

    #[test]
    fn parse_29bit_tahoe_four_ecu_bitmap() {
        // Non-contiguous source addresses 0x11/0x18/0x28/0x40 all enumerate.
        let bus = Bus::new(Addressing::Can29 { tester: 0xF1 });
        let data = "18 DA F1 11 06 41 00 BF DF B9 93\n\
                    18 DA F1 40 06 41 00 80 00 00 01\n\
                    18 DA F1 18 06 41 00 80 00 00 01\n\
                    18 DA F1 28 06 41 00 80 00 00 01";
        let parsed = parse_response_with_bus("0100", data, bus).unwrap();
        let mut ecus: Vec<&String> = parsed.all_controllers.keys().collect();
        ecus.sort();
        assert_eq!(ecus, vec!["11", "18", "28", "40"]);
    }

    #[test]
    fn parse_29bit_mode22_multi_responder_attributes_by_ecu() {
        // Same DID answered by two ECUs must not collapse into one entry.
        let bus = Bus::new(Addressing::Can29 { tester: 0xF1 });
        let data = "18 DA F1 10 05 62 41 82 12 34\n18 DA F1 18 05 62 41 82 56 78";
        let parsed = parse_response_with_bus("224182", data, bus).unwrap();
        assert_eq!(parsed.all_controllers.len(), 2);
        assert_eq!(parsed.all_controllers["10"].data_bytes, vec![0x12, 0x34]);
        assert_eq!(parsed.all_controllers["18"].data_bytes, vec![0x56, 0x78]);
    }

    #[test]
    fn parse_29bit_ecu_name_090a_single_responder() {
        // Physically-targeted 090A: one ECU, multi-frame name string
        // (FF total 0x0D = 3 + "ECM-Engine"; LH7 drops an assembly short of
        // its declared total, so the fixture's PCI must be honest).
        let bus = Bus::new(Addressing::Can29 { tester: 0xF1 });
        let data = "18 DA F1 11 10 0D 49 0A 01 45 43 4D\n\
                    18 DA F1 11 21 2D 45 6E 67 69 6E 65";
        let parsed = parse_response_with_bus("090A", data, bus).unwrap();
        let ctrl = &parsed.all_controllers["11"];
        assert_eq!(ascii(&ctrl.data_bytes[1..]), "ECM-Engine");
    }

    #[test]
    fn guard_11bit_request_header_unchanged_on_the_wire() {
        let bus = Bus::new(Addressing::Can11);
        // Old code sent ATSH<target> where target was 7E0..7E7. Keys are now ecu bytes,
        // but request_header must still produce the exact same 11-bit request address.
        assert_eq!(bus.request_header(Controller::new(0x7E0)), "7E0");
        assert_eq!(bus.request_header(Controller::new(0x7E1)), "7E1");
        assert_eq!(bus.request_header(Controller::new(0x7E7)), "7E7");
        assert_eq!(bus.functional_header(), "7DF");
    }

    #[test]
    fn guard_11bit_parse_data_bytes_unchanged() {
        // Same input a pre-refactor parse would see; data bytes must be identical, only the
        // controller key is the full request id ("7E0", GB14).
        let parsed = parse_response("010C", "7E8 04 41 0C 1A F8").unwrap();
        let ctrl = parsed
            .all_controllers
            .get("7E0")
            .expect("keyed by request id");
        assert_eq!(ctrl.data_bytes, vec![0x1A, 0xF8]);
        assert_eq!(ctrl.controller_id, "7E0");
        assert!(ctrl.is_valid);
    }

    // -------------------------------------------------------
    // Unit tests: helper functions
    // -------------------------------------------------------

    #[test]
    fn test_is_at_command() {
        assert!(is_at_command("ATZ"));
        assert!(is_at_command("ATE0"));
        assert!(is_at_command("ATSH7E0"));
        assert!(is_at_command("STFAC"));
        assert!(!is_at_command("010C"));
        assert!(!is_at_command("03"));
    }

    #[test]
    fn test_is_non_data_response() {
        // SEARCHING… is noise, not a verdict: the data behind a re-search parses.
        assert!(!is_non_data_response(
            "SEARCHING...\r7E8 08 62 F1 88 47 4D 42 4D 32"
        ));
        assert!(parse_response_with_bus(
            "22F188",
            "SEARCHING...\r7E8 08 62 F1 88 47 4D 42 4D 32",
            Bus::new(Addressing::Can11)
        )
        .is_some());
        assert!(parse_response_with_bus(
            "22F188",
            "SEARCHING...\rUNABLE TO CONNECT",
            Bus::new(Addressing::Can11)
        )
        .is_none());
        assert!(is_non_data_response("NO DATA"));
        assert!(is_non_data_response("UNABLE TO CONNECT"));
        assert!(is_non_data_response("?"));
        assert!(is_non_data_response(""));
        assert!(!is_non_data_response("7E8 04 41 0C 1A F8"));
    }

    #[test]
    fn test_parse_command() {
        assert_eq!(parse_command("010C"), ("01".into(), "0C".into()));
        assert_eq!(parse_command("03"), ("03".into(), "".into()));
        assert_eq!(parse_command("0100"), ("01".into(), "00".into()));
        assert_eq!(parse_command("22F40C"), ("22".into(), "F40C".into()));
        assert_eq!(parse_command("0601"), ("06".into(), "01".into()));
    }

    #[test]
    fn test_echo_byte_count() {
        assert_eq!(echo_byte_count("01", "0C"), 2); // 41 0C
        assert_eq!(echo_byte_count("03", ""), 1); // 43
        assert_eq!(echo_byte_count("07", ""), 1); // 47
        assert_eq!(echo_byte_count("22", "F40C"), 3); // 62 F4 0C
        assert_eq!(echo_byte_count("06", "01"), 1); // 46 only (MID in data blocks)
        assert_eq!(echo_byte_count("06", "20"), 2); // 46 20 (PID support)
        assert_eq!(echo_byte_count("09", "02"), 2); // 49 02
    }

    #[test]
    fn test_detect_parse_type() {
        assert_eq!(detect_parse_type("010C"), Some(ParseType::PidResponse));
        assert_eq!(detect_parse_type("0101"), Some(ParseType::MonitorStatus));
        assert_eq!(detect_parse_type("0100"), Some(ParseType::PidSupport));
        assert_eq!(detect_parse_type("0120"), Some(ParseType::PidSupport));
        assert_eq!(detect_parse_type("03"), Some(ParseType::TroubleCodes));
        assert_eq!(detect_parse_type("07"), Some(ParseType::TroubleCodes));
        assert_eq!(detect_parse_type("0601"), Some(ParseType::Mode06));
        assert_eq!(detect_parse_type("ATZ"), None);
    }

    // -------------------------------------------------------
    // AT commands and non-data responses
    // -------------------------------------------------------

    #[test]
    fn test_at_command_returns_none() {
        assert!(parse_response("ATZ", "OK").is_none());
        assert!(parse_response("ATE0", "OK").is_none());
    }

    #[test]
    fn test_no_data_returns_none() {
        assert!(parse_response("010C", "NO DATA").is_none());
        assert!(parse_response("0632", "NO DATA").is_none());
    }

    // -------------------------------------------------------
    // Single-controller PID responses from mock_data
    // -------------------------------------------------------

    #[test]
    fn test_single_controller_pid_response() {
        // "0104" -> "7E8 03 41 04 32" -> data_bytes = [0x32]
        let mock = load_mock_data();
        let data = mock_lines(&mock["01"]["04"]);
        let parsed = parse_response("0104", &data).unwrap();

        assert_eq!(parsed.parse_type, ParseType::PidResponse);
        assert_eq!(parsed.all_controllers.len(), 1);
        assert_eq!(parsed.all_controllers["7E0"].data_bytes, vec![0x32]);
    }

    #[test]
    fn test_single_controller_two_data_bytes() {
        // "0142" -> "7E8 04 41 42 2E E0" -> data_bytes = [0x2E, 0xE0]
        let mock = load_mock_data();
        let data = mock_lines(&mock["01"]["42"]);
        let parsed = parse_response("0142", &data).unwrap();

        assert_eq!(parsed.all_controllers["7E0"].data_bytes, vec![0x2E, 0xE0]);
    }

    // -------------------------------------------------------
    // Multi-controller PID responses
    // -------------------------------------------------------

    #[test]
    fn test_dual_controller_response() {
        // "010C" -> two lines: "7E8 04 41 0C FF FF" and "7E9 04 41 0C FF FF"
        let mock = load_mock_data();
        let data = mock_lines(&mock["01"]["0C"]);
        let parsed = parse_response("010C", &data).unwrap();

        assert_eq!(parsed.all_controllers.len(), 2);
        assert!(parsed.all_controllers.contains_key("7E0"));
        assert!(parsed.all_controllers.contains_key("7E1"));
        // Both should have same data bytes [0xFF, 0xFF]
        assert_eq!(parsed.all_controllers["7E0"].data_bytes, vec![0xFF, 0xFF]);
        assert_eq!(parsed.all_controllers["7E1"].data_bytes, vec![0xFF, 0xFF]);
    }

    #[test]
    fn test_triple_controller_response() {
        // "010D" -> three controllers: 7E8, 7E9, 7EA
        let mock = load_mock_data();
        let data = mock_lines(&mock["01"]["0D"]);
        let parsed = parse_response("010D", &data).unwrap();

        assert_eq!(parsed.all_controllers.len(), 3);
        assert!(parsed.all_controllers.contains_key("7E0"));
        assert!(parsed.all_controllers.contains_key("7E1"));
        assert!(parsed.all_controllers.contains_key("7E2"));
        // All should have [0x6B]
        for ctrl in parsed.all_controllers.values() {
            assert_eq!(ctrl.data_bytes, vec![0x6B]);
        }
    }

    // -------------------------------------------------------
    // ISO-TP multi-frame assembly
    // -------------------------------------------------------

    #[test]
    fn test_iso_tp_two_frame() {
        // "0178" -> first frame + consecutive frame
        // "7E8 10 0B 41 78 0D 0A ED 09"
        // "7E8 21 28 09 E6 0A A8 00 00"
        // After assembly: strip "10 0B" header, strip "21" seq byte
        // Data tokens: 41 78 0D 0A ED 09 28 09 E6 0A A8 00 00
        // Strip echo bytes (41 78) -> 0D 0A ED 09 28 09 E6 0A A8 00 00
        let mock = load_mock_data();
        let data = mock_lines(&mock["01"]["78"]);
        let parsed = parse_response("0178", &data).unwrap();

        assert_eq!(parsed.parse_type, ParseType::PidResponse);
        assert!(parsed.all_controllers.contains_key("7E0"));
        // ISO-TP length = 0x0B = 11 bytes total, minus 2 echo bytes (41 78) = 9 data bytes
        assert_eq!(
            parsed.all_controllers["7E0"].data_bytes,
            vec![0x0D, 0x0A, 0xED, 0x09, 0x28, 0x09, 0xE6, 0x0A, 0xA8]
        );
    }

    #[test]
    fn test_iso_tp_mode09_vin() {
        // "0902" -> VIN: multi-frame ISO-TP
        // "7E8 10 14 49 02 01 31 47 31"
        // "7E8 21 53 59 4E 54 48 30 52"
        // "7E8 22 44 45 46 41 55 4C 54"
        // After assembly, strip echo (49 02): 01 31 47 31 53 59 4E 54 48 30 52 44 45 46 41 55 4C 54
        let mock = load_mock_data();
        let data = mock_lines(&mock["09"]["02"]);
        let parsed = parse_response("0902", &data).unwrap();

        assert_eq!(parsed.parse_type, ParseType::PidResponse);
        let data_bytes = &parsed.all_controllers["7E0"].data_bytes;
        // The data bytes after stripping echo "49 02" should start with 01 (number of data items)
        assert_eq!(data_bytes[0], 0x01);
        // Bytes 1.. are the VIN as ASCII: "1G1SYNTH0RDEFAULT"
        let vin: String = data_bytes[1..]
            .iter()
            .filter(|&&b| b != 0)
            .map(|&b| b as char)
            .collect();
        assert_eq!(vin, "1G1SYNTH0RDEFAULT");
    }

    // -------------------------------------------------------
    // Mode 22 (extended PID) responses
    // -------------------------------------------------------

    #[test]
    fn test_mode22_single_response() {
        // "220404" -> "7E8 05 62 04 04 00 6F"
        // Echo bytes: 62 04 04 (3 bytes) -> data: [0x00, 0x6F]
        let mock = load_mock_data();
        let data = mock_lines(&mock["22"]["0404"]);
        let parsed = parse_response("220404", &data).unwrap();

        assert_eq!(parsed.parse_type, ParseType::PidResponse);
        assert_eq!(parsed.all_controllers["7E0"].data_bytes, vec![0x00, 0x6F]);
    }

    #[test]
    fn test_mode22_no_data() {
        let mock = load_mock_data();
        let data = mock_lines(&mock["22"]["03CA"]);
        assert!(parse_response("2203CA", &data).is_none());
    }

    // -------------------------------------------------------
    // Monitor status (0101)
    // -------------------------------------------------------

    #[test]
    fn test_monitor_status_response() {
        // "0101" -> "7E8 06 41 01 86 07 EF 80" + "7E9 06 41 01 81 00 00 00"
        let mock = load_mock_data();
        let data = mock_lines(&mock["01"]["01"]);
        let parsed = parse_response("0101", &data).unwrap();

        assert_eq!(parsed.parse_type, ParseType::MonitorStatus);
        assert_eq!(parsed.all_controllers.len(), 2);
        // 7E8: strip "41 01" -> [0x86, 0x07, 0xEF, 0x80]
        assert_eq!(
            parsed.all_controllers["7E0"].data_bytes,
            vec![0x86, 0x07, 0xEF, 0x80]
        );
    }

    // -------------------------------------------------------
    // PID support (0120, 0140)
    // -------------------------------------------------------

    #[test]
    fn test_pid_support_response() {
        // "0120" -> "7E8 06 41 20 80 02 20 01"
        // Strip "41 20" -> [0x80, 0x02, 0x20, 0x01]
        let mock = load_mock_data();
        let data = mock_lines(&mock["01"]["20"]);
        let parsed = parse_response("0120", &data).unwrap();

        assert_eq!(parsed.parse_type, ParseType::PidSupport);
        assert_eq!(
            parsed.all_controllers["7E0"].data_bytes,
            vec![0x80, 0x02, 0x20, 0x01]
        );
    }

    // -------------------------------------------------------
    // Trouble codes (Mode 03)
    // -------------------------------------------------------

    #[test]
    fn test_trouble_codes_multi_frame() {
        // "03" -> multi-frame from 7E8 + single from 7E9
        let mock = load_mock_data();
        let data = mock_lines(&mock["03"]["data"]);
        let parsed = parse_response("03", &data).unwrap();

        assert_eq!(parsed.parse_type, ParseType::TroubleCodes);
        assert!(parsed.all_controllers.contains_key("7E0"));
        assert!(parsed.all_controllers.contains_key("7E1"));
    }

    // -------------------------------------------------------
    // Pending trouble codes (Mode 07)
    // -------------------------------------------------------

    #[test]
    fn test_pending_dtcs() {
        let mock = load_mock_data();
        let data = mock_lines(&mock["07"]["data"]);
        let parsed = parse_response("07", &data).unwrap();

        assert_eq!(parsed.parse_type, ParseType::TroubleCodes);
        // Should have 3 controllers: 7E8, 7E9, 7EA
        assert_eq!(parsed.all_controllers.len(), 3);
    }

    // -------------------------------------------------------
    // Mode 06 responses
    // -------------------------------------------------------

    #[test]
    fn test_mode06_multi_frame() {
        // "0631" -> 8-line ISO-TP response, 55 bytes total
        // After echo "46" (1 byte): 54 bytes = 6 × 9-byte blocks
        let mock = load_mock_data();
        let data = mock_lines(&mock["06"]["31"]);
        let parsed = parse_response("0631", &data).unwrap();

        assert_eq!(parsed.parse_type, ParseType::Mode06);
        let data_bytes = &parsed.all_controllers["7E0"].data_bytes;
        assert_eq!(data_bytes.len(), 54);
        // First block starts with MID=0x31
        assert_eq!(data_bytes[0], 0x31);
        // Each 9-byte block should start with MID=0x31
        for i in (0..54).step_by(9) {
            assert_eq!(
                data_bytes[i], 0x31,
                "block at offset {} should start with MID 0x31",
                i
            );
        }
    }

    #[test]
    fn test_mode06_pid_support() {
        // "0620" -> "7E8 06 46 20 C0 00 8C D9" -> PidSupport type
        let mock = load_mock_data();
        let data = mock_lines(&mock["06"]["20"]);
        let parsed = parse_response("0620", &data).unwrap();

        assert_eq!(parsed.parse_type, ParseType::PidSupport);
        // Strip echo "46 20" -> [0xC0, 0x00, 0x8C, 0xD9]
        assert_eq!(
            parsed.all_controllers["7E0"].data_bytes,
            vec![0xC0, 0x00, 0x8C, 0xD9]
        );
    }

    #[test]
    fn test_mode06_no_data() {
        let mock = load_mock_data();
        let data = mock_lines(&mock["06"]["32"]);
        assert!(parse_response("0632", &data).is_none());
    }

    // -------------------------------------------------------
    // Controller selection
    // -------------------------------------------------------

    #[test]
    fn test_select_controller_preferred() {
        let mock = load_mock_data();
        let data = mock_lines(&mock["01"]["0D"]);
        let parsed = parse_response("010D", &data).unwrap();

        let selected = select_controller(&parsed, Some("7E2"), &[]).unwrap();
        assert_eq!(selected.controller_id, "7E2");
    }

    #[test]
    fn test_select_controller_priority() {
        let mock = load_mock_data();
        let data = mock_lines(&mock["01"]["0D"]);
        let parsed = parse_response("010D", &data).unwrap();

        let selected = select_controller(&parsed, None, &["7E2", "7E1", "7E0"]).unwrap();
        assert_eq!(selected.controller_id, "7E2");
    }

    #[test]
    fn test_select_controller_fallback() {
        let mock = load_mock_data();
        let data = mock_lines(&mock["01"]["04"]);
        let parsed = parse_response("0104", &data).unwrap();

        // No preferred, no priority match — should fall back to first valid
        let selected = select_controller(&parsed, Some("7FF"), &["7FF"]).unwrap();
        assert_eq!(selected.controller_id, "7E0");
    }

    // -------------------------------------------------------
    // CAN PCI byte stripping
    // -------------------------------------------------------

    #[test]
    fn test_can_pci_byte_stripped() {
        // "0133" -> "7E8 03 41 33 64"
        // After controller: [03, 41, 33, 64] -> strip PCI "03" -> [41, 33, 64]
        // Strip echo "41 33" -> [0x64]
        let mock = load_mock_data();
        let data = mock_lines(&mock["01"]["33"]);
        let parsed = parse_response("0133", &data).unwrap();
        assert_eq!(parsed.all_controllers["7E0"].data_bytes, vec![0x64]);
    }

    // -------------------------------------------------------
    // parse_response_sniffed — session-less callers (PID editor test data) paste samples
    // from either an 11-bit or 29-bit adapter log; addressing is detected per call.
    // -------------------------------------------------------

    #[test]
    fn sniffed_parses_29bit_sample() {
        // Straight from a 29-bit Jeep adapter log: single-frame Mode 22 response from ECU 0x10.
        let parsed = parse_response_sniffed("22AB08", "18 DA F1 10 04 62 AB 08 2A").unwrap();
        assert_eq!(parsed.all_controllers["10"].data_bytes, vec![0x2A]);
        // Header bytes must NOT leak into the data (the old Can11-only path fell through to a
        // raw-bytes fallback in Swift that treated 18/DA/F1/10 as data).
        assert!(parsed.all_controllers["10"].is_valid);
    }

    #[test]
    fn sniffed_parses_29bit_multiframe_sample() {
        // Multi-frame 29-bit VIN sample (captured Jeep frames).
        let data = "18 DA F1 10 10 14 49 02 01 31 43 34\n\
                    18 DA F1 10 21 4D 4F 43 4B 32 39 53\n\
                    18 DA F1 10 22 42 49 54 30 30 30 31";
        let parsed = parse_response_sniffed("0902", data).unwrap();
        let ascii: String = parsed.all_controllers["10"]
            .data_bytes
            .iter()
            .filter(|b| **b >= 0x20 && **b <= 0x7E)
            .map(|b| *b as char)
            .collect();
        assert_eq!(ascii, "1C4MOCK29SBIT0001");
    }

    // ---- B1: echo-driven multi-PID splitter (fixtures from
    // tests/fixtures/cx_ecusim_multipid_bench_1902.log) ----

    fn m(pairs: &[(&str, u8)]) -> Vec<(String, u8)> {
        pairs.iter().map(|(p, l)| (p.to_string(), *l)).collect()
    }
    fn bus11() -> Bus {
        Bus::new(Addressing::Can11)
    }

    /// Combined single-frame, multi-ECU: `010C0D` benched blob.
    #[test]
    fn chunk_split_single_frame_multi_ecu() {
        let data = "7E8 06 41 0C 3F E3 0D 73 \r7E9 06 41 0C 3F E3 0D 73 \r7EA 03 41 0D 73";
        let out = split_multi_pid_response(&m(&[("010C", 2), ("010D", 1)]), data, bus11()).unwrap();
        let c = &out["010C"];
        assert_eq!(c.all_controllers["7E0"].data_bytes, vec![0x3F, 0xE3]);
        assert_eq!(c.all_controllers["7E1"].data_bytes, vec![0x3F, 0xE3]);
        assert!(
            !c.all_controllers.contains_key("7E2"),
            "7EA didn't answer 0C"
        );
        let d = &out["010D"];
        assert_eq!(d.all_controllers["7E0"].data_bytes, vec![0x73]);
        assert_eq!(d.all_controllers["7E2"].data_bytes, vec![0x73]);
        // Member responses are shaped like solo parses.
        assert_eq!(c.parse_type, ParseType::PidResponse);
        assert_eq!(c.all_controllers["7E0"].raw_hex, "41 0C 3F E3");
    }

    /// Reassembled multi-frame + per-ECU subsets + silent omission, exactly
    /// as benched: 6-PID request, 7E8 answers 5 (PID 11 missing), 7E9 a
    /// 3-PID subset, 7EA one PID. Note 7E8's payload contains `0F 41` — a
    /// 0x41 DATA byte mid-scan that must not be mistaken for a mode echo.
    #[test]
    fn chunk_split_multiframe_subsets_and_omission() {
        let data = "7E8 10 0C 41 0C 3F E3 0D 73 \r7E8 21 05 4A 0F 41 04 32 00 \r7E9 10 08 41 0C 3F E3 0D 73 \r7E9 21 05 4A 00 00 00 00 00 \r7EA 03 41 0D 73";
        let members = m(&[
            ("010C", 2),
            ("010D", 1),
            ("0105", 1),
            ("010F", 1),
            ("0111", 1),
            ("0104", 1),
        ]);
        let out = split_multi_pid_response(&members, data, bus11()).unwrap();
        assert_eq!(
            out["010C"].all_controllers["7E0"].data_bytes,
            vec![0x3F, 0xE3]
        );
        assert_eq!(out["0105"].all_controllers["7E0"].data_bytes, vec![0x4A]);
        assert_eq!(out["010F"].all_controllers["7E0"].data_bytes, vec![0x41]);
        assert_eq!(out["0104"].all_controllers["7E0"].data_bytes, vec![0x32]);
        // 7E9 subset: 0C/0D/05 only (ISO-TP len 8 truncates its padding).
        assert_eq!(out["0105"].all_controllers["7E1"].data_bytes, vec![0x4A]);
        assert!(!out["010F"].all_controllers.contains_key("7E1"));
        // 7EA answered only 0D.
        assert_eq!(out["010D"].all_controllers["7E2"].data_bytes, vec![0x73]);
        // Silent omission: no controller echoed 11 → member absent, no error.
        assert!(!out.contains_key("0111"));
    }

    /// CF padding zeros after the last member stop the scan cleanly.
    #[test]
    fn chunk_split_stops_at_padding() {
        let data = "7E8 06 41 0C 3F E3 0D 73 00 00";
        let out = split_multi_pid_response(&m(&[("010C", 2), ("010D", 1)]), data, bus11()).unwrap();
        assert_eq!(
            out["010C"].all_controllers["7E0"].data_bytes,
            vec![0x3F, 0xE3]
        );
        assert_eq!(out["010D"].all_controllers["7E0"].data_bytes, vec![0x73]);
    }

    /// Response order ≠ request order — echo-keyed, not position-keyed.
    #[test]
    fn chunk_split_out_of_order_response() {
        let data = "7E8 06 41 0D 73 0C 3F E3";
        let out = split_multi_pid_response(&m(&[("010C", 2), ("010D", 1)]), data, bus11()).unwrap();
        assert_eq!(
            out["010C"].all_controllers["7E0"].data_bytes,
            vec![0x3F, 0xE3]
        );
        assert_eq!(out["010D"].all_controllers["7E0"].data_bytes, vec![0x73]);
    }

    /// Duplicate echo from one controller = a mis-slice symptom → Validation.
    #[test]
    fn chunk_split_duplicate_echo_fails_validation() {
        let data = "7E8 06 41 0D 73 0D 73";
        let err = split_multi_pid_response(&m(&[("010D", 1)]), data, bus11()).unwrap_err();
        assert!(matches!(err, ChunkSplitError::Validation(_)), "{err:?}");
    }

    /// Unknown echo mid-scan (stale length walked into foreign bytes) → Validation.
    #[test]
    fn chunk_split_unknown_echo_fails_validation() {
        // 0C sliced with a stale length of 1 lands on 0xE3 as the next echo.
        let data = "7E8 06 41 0C 3F E3 0D 73";
        let err =
            split_multi_pid_response(&m(&[("010C", 1), ("010D", 1)]), data, bus11()).unwrap_err();
        assert!(matches!(err, ChunkSplitError::Validation(_)), "{err:?}");
    }

    /// Member data running past the payload end → Validation.
    #[test]
    fn chunk_split_truncated_data_fails_validation() {
        let data = "7E8 04 41 0C 3F";
        let err = split_multi_pid_response(&m(&[("010C", 2)]), data, bus11()).unwrap_err();
        assert!(matches!(err, ChunkSplitError::Validation(_)), "{err:?}");
    }

    /// Non-41 payload (e.g. 7F NRC) → Validation; NO DATA → NonData.
    #[test]
    fn chunk_split_nrc_and_no_data() {
        let err =
            split_multi_pid_response(&m(&[("010C", 2)]), "7E8 03 7F 01 12", bus11()).unwrap_err();
        assert!(matches!(err, ChunkSplitError::Validation(_)), "{err:?}");
        let err = split_multi_pid_response(&m(&[("010C", 2)]), "NO DATA", bus11()).unwrap_err();
        assert_eq!(err, ChunkSplitError::NonData);
    }

    // ---- B3: Mode 22 multi-DID splitter (golden fixture = the Mustang S197
    // bench response to `22F40CF405`, 2026-07-31, engine idling) ----

    /// Verbatim Mustang capture: multi-frame `62` carrying RPM (F40C, 2 bytes,
    /// 0x0B73/4 ≈ 733 rpm) then ECT (F405, 1 byte, 0x7A−40 = 82 °C).
    #[test]
    fn chunk22_split_mustang_golden() {
        let data = "7E8 10 08 62 F4 0C 0B 73 F4 \r7E8 21 05 7A 00 00 00 00 00";
        let out =
            split_multi_pid_response(&m(&[("22F405", 1), ("22F40C", 2)]), data, bus11()).unwrap();
        let rpm = &out["22F40C"];
        assert_eq!(rpm.all_controllers["7E0"].data_bytes, vec![0x0B, 0x73]);
        assert_eq!(rpm.all_controllers["7E0"].raw_hex, "62 F4 0C 0B 73");
        assert_eq!(rpm.parse_type, ParseType::PidResponse);
        let ect = &out["22F405"];
        assert_eq!(ect.all_controllers["7E0"].data_bytes, vec![0x7A]);
        assert_eq!(ect.all_controllers["7E0"].raw_hex, "62 F4 05 7A");
    }

    /// Negative response to the multi-DID shape (`7F 22 …`) = Validation —
    /// the demux maps it to Refused (solos, no length invalidation).
    #[test]
    fn chunk22_split_negative_response_fails_validation() {
        let err = split_multi_pid_response(
            &m(&[("22F405", 1), ("22F40C", 2)]),
            "7E8 03 7F 22 31",
            bus11(),
        )
        .unwrap_err();
        assert!(matches!(err, ChunkSplitError::Validation(_)), "{err:?}");
    }

    /// A 0x00 lead byte stops the scan (CF padding) — trailing pad bytes
    /// after the last member never read as echoes.
    #[test]
    fn chunk22_split_stops_at_padding() {
        // Same golden but with only the RPM member requested: the scan takes
        // F40C then hits F4 05 — unknown echo → Validation (mis-slice guard).
        let data = "7E8 10 08 62 F4 0C 0B 73 F4 \r7E8 21 05 7A 00 00 00 00 00";
        let err = split_multi_pid_response(&m(&[("22F40C", 2)]), data, bus11()).unwrap_err();
        assert!(matches!(err, ChunkSplitError::Validation(_)), "{err:?}");

        // Clean padding stop: single member, pad bytes after its data.
        let data = "7E8 05 62 F4 0C 0B 73"; // exact, no padding needed
        let out = split_multi_pid_response(&m(&[("22F40C", 2)]), data, bus11()).unwrap();
        assert_eq!(
            out["22F40C"].all_controllers["7E0"].data_bytes,
            vec![0x0B, 0x73]
        );
    }

    /// B2 probe parsing: both PIDs → capable; one / NRC / NO DATA / noise → serial.
    #[test]
    fn chunk_probe_response_parsing() {
        // Both (the benched Malibu answer) — capable.
        let probe = |text: &str| chunk_probe_capable(&text_payloads("010C0D", text, bus11()));
        assert!(probe("7E8 06 41 0C 3F E3 0D 73 \r7E9 06 41 0C 3F E3 0D 73"));
        // Only one PID answered → serial.
        assert!(!probe("7E8 04 41 0C 3F E3"));
        assert!(!probe("7EA 03 41 0D 73"));
        // NRC, NO DATA, adapter noise → serial.
        assert!(!probe("7E8 03 7F 01 12"));
        assert!(!probe("NO DATA"));
        assert!(!probe("?"));
        assert!(!probe(""));
    }

    #[test]
    fn sniffed_11bit_and_headerless_unchanged() {
        // 11-bit frame parses exactly as the default path does.
        let parsed = parse_response_sniffed("010C", "7E8 04 41 0C 1A F8").unwrap();
        assert_eq!(parsed.all_controllers["7E0"].data_bytes, vec![0x1A, 0xF8]);
        // Headerless raw bytes: no recognizable header → sniff falls back to Can11, which
        // attributes the line to the "DEFAULT" controller with echo stripped — byte-identical
        // to the pre-sniff behavior.
        let parsed = parse_response_sniffed("22AB08", "62 AB 08 2A").unwrap();
        assert_eq!(parsed.all_controllers["DEFAULT"].data_bytes, vec![0x2A]);
    }
    /// Gladiator bench 2026-08-02: Stellantis pads single-frame responses
    /// with STALE bytes (not 00). The splitter must stop at the PCI length —
    /// walking into the 0x56 pad froze RPM under STN periodic.
    #[test]
    fn chunk_split_ignores_non_zero_frame_padding() {
        let members = vec![("010C".to_string(), 2u8), ("010D".to_string(), 1u8)];
        let line = "7E8 06 41 0C 0C 68 0D 00 56";
        let out = split_multi_pid_response(
            &members,
            line,
            Bus::new(crate::addressing::Addressing::Can11),
        )
        .expect("stale pad byte must not fail the frame");
        let rpm = &out["010C"].all_controllers["7E0"];
        assert_eq!(rpm.data_bytes, vec![0x0C, 0x68]);
        let speed = &out["010D"].all_controllers["7E0"];
        assert_eq!(speed.data_bytes, vec![0x00]);
    }
}
