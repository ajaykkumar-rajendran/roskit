//! CBOR encoder for structured ROS messages

use bytes::Bytes;
use serde::Serialize;

/// Encode any serializable message to CBOR
pub fn encode_message_cbor<T: Serialize>(
    message: &T,
) -> Result<Bytes, Box<dyn std::error::Error + Send + Sync>> {
    let mut buf = Vec::new();
    ciborium::into_writer(message, &mut buf)?;
    Ok(Bytes::from(buf))
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde::Deserialize;

    #[derive(Debug, Serialize, Deserialize, PartialEq)]
    struct TestMessage {
        x: f64,
        y: f64,
        z: f64,
    }

    #[test]
    fn test_encode_decode_cbor() {
        let msg = TestMessage {
            x: 1.0,
            y: 2.0,
            z: 3.0,
        };

        let encoded = encode_message_cbor(&msg).unwrap();
        assert!(!encoded.is_empty());

        // Verify we can decode it back
        let decoded: TestMessage = ciborium::from_reader(&encoded[..]).unwrap();
        assert_eq!(msg, decoded);
    }
}
