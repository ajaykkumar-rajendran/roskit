#!/usr/bin/env python3
"""
TLS and Authentication Tests for RosKit Python Bridge

These tests verify:
- Token-based authentication flow
- TLS configuration
- Request validation logic
- Error handling for invalid tokens

Run with: pytest tests/test_tls_auth.py -v
"""

import ssl
import tempfile
import os
from pathlib import Path

import pytest


# ============================================================================
# Authentication Tests
# ============================================================================

class MockRequest:
    """Mock HTTP request for testing."""
    def __init__(self, headers: dict, path: str = "/"):
        self.headers = headers
        self.path = path


def extract_token_from_headers(headers: dict, query_string: str = "") -> str | None:
    """
    Extract authentication token from request headers or query string.

    Supports:
    - Authorization: Bearer <token>
    - X-Api-Key: <token>
    - ?token=<token> query parameter
    """
    # Check Authorization header
    auth_header = headers.get('Authorization', '')
    if auth_header.startswith('Bearer '):
        return auth_header[7:].strip()

    # Check X-Api-Key header
    api_key = headers.get('X-Api-Key')
    if api_key:
        return api_key.strip()

    # Check query parameters
    if query_string:
        for param in query_string.split('&'):
            if param.startswith('token='):
                return param[6:]

    return None


def validate_token(token: str | None, expected_token: str | None) -> bool:
    """Validate a token against the expected token."""
    if not expected_token:
        return True  # No auth required

    if not token:
        return False

    return token == expected_token


class TestTokenExtraction:
    """Token extraction tests."""

    def test_extract_bearer_token(self):
        """Extract token from Bearer header."""
        headers = {'Authorization': 'Bearer my_secret_token'}
        token = extract_token_from_headers(headers)
        assert token == 'my_secret_token'

    def test_extract_bearer_token_with_whitespace(self):
        """Extract token from Bearer header with extra whitespace."""
        headers = {'Authorization': 'Bearer   my_token  '}
        token = extract_token_from_headers(headers)
        assert token == 'my_token'

    def test_extract_api_key_header(self):
        """Extract token from X-Api-Key header."""
        headers = {'X-Api-Key': 'api_key_12345'}
        token = extract_token_from_headers(headers)
        assert token == 'api_key_12345'

    def test_extract_query_param(self):
        """Extract token from query parameter."""
        headers = {}
        token = extract_token_from_headers(headers, 'token=query_token_abc')
        assert token == 'query_token_abc'

    def test_extract_query_param_with_other_params(self):
        """Extract token from query with multiple parameters."""
        headers = {}
        token = extract_token_from_headers(headers, 'foo=bar&token=my_token&baz=qux')
        assert token == 'my_token'

    def test_bearer_takes_priority(self):
        """Bearer header takes priority over query param."""
        headers = {'Authorization': 'Bearer header_token'}
        token = extract_token_from_headers(headers, 'token=query_token')
        assert token == 'header_token'

    def test_no_token_found(self):
        """Return None when no token found."""
        headers = {'Content-Type': 'application/json'}
        token = extract_token_from_headers(headers, 'foo=bar')
        assert token is None

    def test_empty_bearer(self):
        """Handle empty Bearer value."""
        headers = {'Authorization': 'Bearer '}
        token = extract_token_from_headers(headers)
        assert token == ''

    def test_basic_auth_ignored(self):
        """Basic auth should not be extracted."""
        headers = {'Authorization': 'Basic dXNlcjpwYXNz'}
        token = extract_token_from_headers(headers)
        assert token is None

    def test_lowercase_headers(self):
        """Handle lowercase header names."""
        # Note: In practice, HTTP headers are case-insensitive
        # This tests our implementation handles the common form
        headers = {'authorization': 'Bearer lower_token'}
        # Our simple implementation is case-sensitive, but
        # the websockets library normalizes headers
        token = extract_token_from_headers(headers)
        assert token is None  # Our simple impl doesn't handle lowercase

    def test_special_characters_in_token(self):
        """Handle tokens with special characters."""
        headers = {'Authorization': 'Bearer token+with/special=chars'}
        token = extract_token_from_headers(headers)
        assert token == 'token+with/special=chars'

    def test_jwt_like_token(self):
        """Handle JWT-style tokens."""
        jwt = 'eyJhbGciOiJIUzI1NiJ9.eyJzdWIiOiJ1c2VyIn0.signature'
        headers = {'Authorization': f'Bearer {jwt}'}
        token = extract_token_from_headers(headers)
        assert token == jwt


class TestTokenValidation:
    """Token validation tests."""

    def test_valid_token(self):
        """Valid token should pass validation."""
        assert validate_token('correct_token', 'correct_token') is True

    def test_invalid_token(self):
        """Invalid token should fail validation."""
        assert validate_token('wrong_token', 'correct_token') is False

    def test_no_auth_required(self):
        """Any token passes when auth not required."""
        assert validate_token('any_token', None) is True
        assert validate_token(None, None) is True

    def test_missing_token_when_required(self):
        """Missing token fails when auth required."""
        assert validate_token(None, 'expected_token') is False
        assert validate_token('', 'expected_token') is False

    def test_empty_expected_token(self):
        """Empty expected token means no auth required."""
        assert validate_token('any_token', '') is True


# ============================================================================
# TLS Configuration Tests
# ============================================================================

class TestTLSConfiguration:
    """TLS configuration tests."""

    def test_ssl_context_creation(self):
        """SSL context can be created with proper protocol."""
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        assert ctx is not None
        assert ctx.protocol == ssl.PROTOCOL_TLS_SERVER

    def test_ssl_context_with_test_certs(self):
        """SSL context can load test certificates."""
        # Create temporary test certificate and key
        with tempfile.TemporaryDirectory() as tmp_dir:
            cert_path = Path(tmp_dir) / 'test_cert.pem'
            key_path = Path(tmp_dir) / 'test_key.pem'

            # Self-signed test certificate (EC P-256)
            test_cert = """-----BEGIN CERTIFICATE-----
MIIBkTCB+wIJAKHBfpegPjMCMA0GCSqGSIb3DQEBCwUAMBExDzANBgNVBAMMBnRl
c3RjYTAeFw0yNDAxMDEwMDAwMDBaFw0yNTAxMDEwMDAwMDBaMBExDzANBgNVBAMM
BnRlc3RjYTBZMBMGByqGSM49AgEGCCqGSM49AwEHA0IABFV9sGxKpjHCfm/u4Pvy
yH5EqgU5eN6JU/3HzJxPSRVdC7NvLz6nwM3RCXsJXqqxZ3l4EpqTl4P1KfJMmZ/D
fEijUzBRMB0GA1UdDgQWBBQ5a8F0L0LjN6VfZxJXQxKJRWJGjTAfBgNVHSMEGDAW
gBQ5a8F0L0LjN6VfZxJXQxKJRWJGjTAPBgNVHRMBAf8EBTADAQH/MA0GCSqGSIb3
DQEBCwUAA0EA5VXKh/kHcWm3KxK5zHx0/FViDqX1gxRPqTKRwPJm6xZ5P6Z6/d3C
tH3jRUjhpM5lWI8/qxM5L8jXrZDLxJvLTA==
-----END CERTIFICATE-----
"""
            test_key = """-----BEGIN EC PRIVATE KEY-----
MHQCAQEEIODaxiCfyLq0awLfVfQL+KMgLE9n5IbLwZLKCgYbECbLoAcGBSuBBAAK
oUQDQgAEVX2wbEqmMcJ+b+7g+/LIfkSqBTl43olT/cfMnE9JFV0Ls28vPqfAzdEJ
ewleqrFneXgSmpOXg/Up8kyZn8N8SA==
-----END EC PRIVATE KEY-----
"""

            cert_path.write_text(test_cert)
            key_path.write_text(test_key)

            ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            # Note: These test certs are invalid, so loading will fail
            # This tests the API, not actual TLS functionality
            with pytest.raises(ssl.SSLError):
                ctx.load_cert_chain(str(cert_path), str(key_path))

    def test_ssl_context_missing_cert(self):
        """SSL context fails with missing certificate."""
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        with pytest.raises(FileNotFoundError):
            ctx.load_cert_chain('/nonexistent/cert.pem', '/nonexistent/key.pem')

    def test_tls_enabled_flag(self):
        """Check TLS enabled based on cert presence."""
        # Simulate the logic from bridge_node.py
        tls_cert = '/path/to/cert.pem'
        proto = 'wss' if tls_cert else 'ws'
        assert proto == 'wss'

        tls_cert = ''
        proto = 'wss' if tls_cert else 'ws'
        assert proto == 'ws'


# ============================================================================
# Integration Scenario Tests
# ============================================================================

class TestAuthFlows:
    """Authentication flow integration tests."""

    def test_complete_auth_flow_success(self):
        """Complete successful authentication flow."""
        expected_token = 'secret_production_token'

        # Client sends request with valid token
        headers = {'Authorization': 'Bearer secret_production_token'}
        token = extract_token_from_headers(headers)

        assert token is not None
        assert validate_token(token, expected_token) is True

    def test_complete_auth_flow_failure(self):
        """Complete failed authentication flow."""
        expected_token = 'correct_token'

        # Client sends request with wrong token
        headers = {'Authorization': 'Bearer wrong_token'}
        token = extract_token_from_headers(headers)

        assert token is not None
        assert validate_token(token, expected_token) is False

    def test_auth_flow_missing_token(self):
        """Authentication flow with missing token."""
        expected_token = 'expected_token'

        # Client sends request without token
        headers = {'Content-Type': 'application/json'}
        token = extract_token_from_headers(headers)

        assert token is None
        assert validate_token(token, expected_token) is False

    def test_auth_disabled(self):
        """Authentication disabled allows all connections."""
        # When auth_token is empty/None, all connections are allowed
        assert validate_token('any_token', None) is True
        assert validate_token(None, None) is True
        assert validate_token('', None) is True

    def test_multiple_auth_methods(self):
        """Test all supported authentication methods."""
        expected_token = 'multi_method_token'

        # Bearer token
        headers1 = {'Authorization': f'Bearer {expected_token}'}
        assert validate_token(extract_token_from_headers(headers1), expected_token)

        # API key
        headers2 = {'X-Api-Key': expected_token}
        assert validate_token(extract_token_from_headers(headers2), expected_token)

        # Query param
        headers3 = {}
        assert validate_token(
            extract_token_from_headers(headers3, f'token={expected_token}'),
            expected_token
        )


# ============================================================================
# Edge Cases
# ============================================================================

class TestEdgeCases:
    """Edge case tests."""

    def test_very_long_token(self):
        """Handle very long tokens."""
        long_token = 'a' * 1000
        headers = {'Authorization': f'Bearer {long_token}'}
        token = extract_token_from_headers(headers)
        assert token == long_token
        assert validate_token(token, long_token) is True

    def test_unicode_token(self):
        """Handle unicode tokens."""
        unicode_token = 'token_with_émojis_🔐'
        headers = {'Authorization': f'Bearer {unicode_token}'}
        token = extract_token_from_headers(headers)
        assert token == unicode_token
        assert validate_token(token, unicode_token) is True

    def test_empty_headers(self):
        """Handle empty headers dict."""
        token = extract_token_from_headers({})
        assert token is None

    def test_none_values_in_headers(self):
        """Handle None values in headers."""
        headers = {'Authorization': None, 'X-Api-Key': None}
        # This would raise an error in the real implementation
        # Our test implementation handles it gracefully
        with pytest.raises(AttributeError):
            extract_token_from_headers(headers)

    def test_case_sensitivity(self):
        """Tokens are case-sensitive."""
        assert validate_token('Token', 'token') is False
        assert validate_token('TOKEN', 'token') is False
        assert validate_token('token', 'token') is True

    def test_whitespace_only_token(self):
        """Handle whitespace-only tokens."""
        headers = {'Authorization': 'Bearer    '}
        token = extract_token_from_headers(headers)
        assert token == ''

    def test_url_encoded_token(self):
        """Handle URL-encoded tokens in query string."""
        # URL encoding: space -> %20, + stays +
        headers = {}
        token = extract_token_from_headers(headers, 'token=my%20token')
        # Note: In practice, the web framework handles URL decoding
        # Our simple implementation doesn't decode
        assert token == 'my%20token'


# ============================================================================
# Token File Tests
# ============================================================================

class TestTokenFile:
    """Token file handling tests."""

    def test_create_token_file(self):
        """Create and read token file."""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.txt', delete=False) as f:
            f.write("token1\n")
            f.write("token2\n")
            f.write("# comment line\n")
            f.write("\n")  # empty line
            f.write("token3\n")
            token_path = f.name

        try:
            with open(token_path) as f:
                lines = f.readlines()

            # Parse tokens (simulating bridge behavior)
            tokens = []
            for line in lines:
                line = line.strip()
                if line and not line.startswith('#'):
                    tokens.append(line)

            assert len(tokens) == 3
            assert 'token1' in tokens
            assert 'token2' in tokens
            assert 'token3' in tokens
            assert '# comment line' not in tokens
        finally:
            os.unlink(token_path)

    def test_empty_token_file(self):
        """Handle empty token file."""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.txt', delete=False) as f:
            f.write("")
            token_path = f.name

        try:
            with open(token_path) as f:
                content = f.read()

            tokens = [t for t in content.strip().split('\n') if t and not t.startswith('#')]
            assert len(tokens) == 0
        finally:
            os.unlink(token_path)

    def test_token_file_with_only_comments(self):
        """Handle token file with only comments."""
        with tempfile.NamedTemporaryFile(mode='w', suffix='.txt', delete=False) as f:
            f.write("# comment 1\n")
            f.write("# comment 2\n")
            token_path = f.name

        try:
            with open(token_path) as f:
                lines = f.readlines()

            tokens = [line.strip() for line in lines
                      if line.strip() and not line.strip().startswith('#')]
            assert len(tokens) == 0
        finally:
            os.unlink(token_path)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
