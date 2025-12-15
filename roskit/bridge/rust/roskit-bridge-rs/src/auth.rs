//! Authentication module for RosKit Bridge
//!
//! Provides simple token-based authentication for WebSocket connections.

use std::collections::HashSet;
use std::sync::Arc;
use parking_lot::RwLock;
use tracing::{debug, warn};

/// Authentication configuration
#[derive(Debug, Clone)]
pub struct AuthConfig {
    /// Whether authentication is enabled
    pub enabled: bool,
    /// Valid API tokens (if empty, all tokens are rejected when auth is enabled)
    pub tokens: HashSet<String>,
    /// Optional token file path (tokens loaded on startup)
    pub token_file: Option<String>,
}

impl Default for AuthConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            tokens: HashSet::new(),
            token_file: None,
        }
    }
}

impl AuthConfig {
    /// Create a new auth config with authentication enabled
    pub fn with_tokens(tokens: Vec<String>) -> Self {
        Self {
            enabled: true,
            tokens: tokens.into_iter().collect(),
            token_file: None,
        }
    }

    /// Create a new auth config with a token file
    pub fn with_token_file(path: impl Into<String>) -> Self {
        Self {
            enabled: true,
            tokens: HashSet::new(),
            token_file: Some(path.into()),
        }
    }

    /// Load tokens from the configured token file
    pub fn load_tokens_from_file(&mut self) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        if let Some(ref path) = self.token_file {
            let content = std::fs::read_to_string(path)
                .map_err(|e| format!("Failed to read token file '{}': {}", path, e))?;

            for line in content.lines() {
                let token = line.trim();
                if !token.is_empty() && !token.starts_with('#') {
                    self.tokens.insert(token.to_string());
                }
            }

            debug!("Loaded {} tokens from file", self.tokens.len());
        }
        Ok(())
    }
}

/// Authentication manager
pub struct AuthManager {
    config: RwLock<AuthConfig>,
}

impl AuthManager {
    /// Create a new authentication manager
    pub fn new(config: AuthConfig) -> Self {
        Self {
            config: RwLock::new(config),
        }
    }

    /// Check if authentication is enabled
    pub fn is_enabled(&self) -> bool {
        self.config.read().enabled
    }

    /// Validate a token
    pub fn validate_token(&self, token: &str) -> bool {
        let config = self.config.read();

        if !config.enabled {
            return true;
        }

        if config.tokens.is_empty() {
            warn!("Authentication enabled but no tokens configured");
            return false;
        }

        let is_valid = config.tokens.contains(token);

        if !is_valid {
            warn!("Invalid authentication token attempted");
        }

        is_valid
    }

    /// Add a token to the valid tokens set
    pub fn add_token(&self, token: impl Into<String>) {
        self.config.write().tokens.insert(token.into());
    }

    /// Remove a token from the valid tokens set
    pub fn remove_token(&self, token: &str) -> bool {
        self.config.write().tokens.remove(token)
    }

    /// Reload tokens from the configured file
    pub fn reload_tokens(&self) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        self.config.write().load_tokens_from_file()
    }
}

/// Extract token from WebSocket upgrade request headers
///
/// Supports the following authentication methods:
/// - `Authorization: Bearer <token>` header
/// - `X-Api-Key: <token>` header
/// - `?token=<token>` query parameter
pub fn extract_token_from_request(
    headers: &[(String, String)],
    query_params: Option<&str>,
) -> Option<String> {
    // Check Authorization header
    for (name, value) in headers {
        if name.to_lowercase() == "authorization" {
            if let Some(token) = value.strip_prefix("Bearer ") {
                return Some(token.trim().to_string());
            }
        }
        if name.to_lowercase() == "x-api-key" {
            return Some(value.trim().to_string());
        }
    }

    // Check query parameters
    if let Some(query) = query_params {
        for param in query.split('&') {
            if let Some((key, value)) = param.split_once('=') {
                if key == "token" {
                    return Some(value.to_string());
                }
            }
        }
    }

    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_auth_disabled_by_default() {
        let config = AuthConfig::default();
        let manager = AuthManager::new(config);

        assert!(!manager.is_enabled());
        assert!(manager.validate_token("any_token"));
    }

    #[test]
    fn test_auth_with_tokens() {
        let config = AuthConfig::with_tokens(vec!["valid_token".to_string()]);
        let manager = AuthManager::new(config);

        assert!(manager.is_enabled());
        assert!(manager.validate_token("valid_token"));
        assert!(!manager.validate_token("invalid_token"));
    }

    #[test]
    fn test_add_remove_tokens() {
        let config = AuthConfig::with_tokens(vec!["token1".to_string()]);
        let manager = AuthManager::new(config);

        assert!(manager.validate_token("token1"));
        assert!(!manager.validate_token("token2"));

        manager.add_token("token2");
        assert!(manager.validate_token("token2"));

        assert!(manager.remove_token("token2"));
        assert!(!manager.validate_token("token2"));
    }

    #[test]
    fn test_extract_token_from_bearer_header() {
        let headers = vec![
            ("Authorization".to_string(), "Bearer my_token".to_string()),
        ];
        assert_eq!(
            extract_token_from_request(&headers, None),
            Some("my_token".to_string())
        );
    }

    #[test]
    fn test_extract_token_from_api_key_header() {
        let headers = vec![
            ("X-Api-Key".to_string(), "my_api_key".to_string()),
        ];
        assert_eq!(
            extract_token_from_request(&headers, None),
            Some("my_api_key".to_string())
        );
    }

    #[test]
    fn test_extract_token_from_query_param() {
        let headers: Vec<(String, String)> = vec![];
        assert_eq!(
            extract_token_from_request(&headers, Some("token=query_token&other=value")),
            Some("query_token".to_string())
        );
    }

    #[test]
    fn test_extract_token_none_found() {
        let headers: Vec<(String, String)> = vec![];
        assert_eq!(extract_token_from_request(&headers, None), None);
    }
}
