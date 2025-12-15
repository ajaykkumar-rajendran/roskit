//! TLS configuration and utilities for RosKit Bridge
//!
//! Provides TLS/SSL support for secure WebSocket connections (wss://).

use rustls::pki_types::{CertificateDer, PrivateKeyDer};
use rustls::ServerConfig;
use std::fs::File;
use std::io::BufReader;
use std::path::Path;
use std::sync::Arc;
use tokio_rustls::TlsAcceptor;
use tracing::info;

/// TLS configuration for the server
#[derive(Debug, Clone)]
pub struct TlsConfig {
    /// Path to the certificate file (PEM format)
    pub cert_path: String,
    /// Path to the private key file (PEM format)
    pub key_path: String,
}

impl TlsConfig {
    /// Create a new TLS configuration
    pub fn new(cert_path: impl Into<String>, key_path: impl Into<String>) -> Self {
        Self {
            cert_path: cert_path.into(),
            key_path: key_path.into(),
        }
    }

    /// Build a TLS acceptor from the configuration
    pub fn build_acceptor(&self) -> Result<TlsAcceptor, Box<dyn std::error::Error + Send + Sync>> {
        info!("Loading TLS certificate from: {}", self.cert_path);
        info!("Loading TLS private key from: {}", self.key_path);

        let certs = load_certs(&self.cert_path)?;
        let key = load_private_key(&self.key_path)?;

        let config = ServerConfig::builder()
            .with_no_client_auth()
            .with_single_cert(certs, key)?;

        Ok(TlsAcceptor::from(Arc::new(config)))
    }
}

/// Load certificates from a PEM file
fn load_certs(
    path: &str,
) -> Result<Vec<CertificateDer<'static>>, Box<dyn std::error::Error + Send + Sync>> {
    let file = File::open(Path::new(path))
        .map_err(|e| format!("Failed to open certificate file '{}': {}", path, e))?;
    let mut reader = BufReader::new(file);

    let certs: Vec<CertificateDer<'static>> = rustls_pemfile::certs(&mut reader)
        .filter_map(|result| result.ok())
        .collect();

    if certs.is_empty() {
        return Err(format!("No valid certificates found in '{}'", path).into());
    }

    info!("Loaded {} certificate(s)", certs.len());
    Ok(certs)
}

/// Load a private key from a PEM file
fn load_private_key(
    path: &str,
) -> Result<PrivateKeyDer<'static>, Box<dyn std::error::Error + Send + Sync>> {
    let file = File::open(Path::new(path))
        .map_err(|e| format!("Failed to open private key file '{}': {}", path, e))?;
    let mut reader = BufReader::new(file);

    // Try to read PKCS#8 private keys first
    let keys: Vec<_> = rustls_pemfile::pkcs8_private_keys(&mut reader)
        .filter_map(|result| result.ok())
        .collect();

    if !keys.is_empty() {
        info!("Loaded PKCS#8 private key");
        return Ok(PrivateKeyDer::Pkcs8(keys.into_iter().next().unwrap()));
    }

    // Try RSA private keys
    let file = File::open(Path::new(path))?;
    let mut reader = BufReader::new(file);
    let keys: Vec<_> = rustls_pemfile::rsa_private_keys(&mut reader)
        .filter_map(|result| result.ok())
        .collect();

    if !keys.is_empty() {
        info!("Loaded RSA private key");
        return Ok(PrivateKeyDer::Pkcs1(keys.into_iter().next().unwrap()));
    }

    // Try EC private keys
    let file = File::open(Path::new(path))?;
    let mut reader = BufReader::new(file);
    let keys: Vec<_> = rustls_pemfile::ec_private_keys(&mut reader)
        .filter_map(|result| result.ok())
        .collect();

    if !keys.is_empty() {
        info!("Loaded EC private key");
        return Ok(PrivateKeyDer::Sec1(keys.into_iter().next().unwrap()));
    }

    Err(format!("No valid private key found in '{}'", path).into())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tls_config_creation() {
        let config = TlsConfig::new("/path/to/cert.pem", "/path/to/key.pem");
        assert_eq!(config.cert_path, "/path/to/cert.pem");
        assert_eq!(config.key_path, "/path/to/key.pem");
    }
}
