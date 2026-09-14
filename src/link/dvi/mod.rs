//! OBDX Pro DVI (LH6). `codec` is the pure byte protocol (DV0); `handler`
//! the `LinkHandler` over it (DV1).
pub mod codec;
pub mod handler;
pub mod periodic;
