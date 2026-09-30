"""Small HTTP helpers used by the navigation request handlers."""

import json
import time


def read_request_body(handler):
    """Read and decode a request body without assuming Content-Length exists."""
    try:
        content_length = int(handler.headers.get("Content-Length", 0))
    except (TypeError, ValueError):
        content_length = 0
    body = handler.rfile.read(content_length) if content_length > 0 else b""
    return body, body.decode("utf-8", errors="replace") if body else ""


def parse_json_body(body):
    """Decode a JSON request body, treating an empty body as an empty object."""
    if not body:
        return {}
    return json.loads(body.decode("utf-8"))


def send_text(handler, status, message, reason=None, append_newline=True):
    """Send a UTF-8 text response with a correct content length."""
    text = str(message) + ("\n" if append_newline else "")
    payload = text.encode("utf-8")
    if reason:
        handler.send_response(status, reason)
    else:
        handler.send_response(status)
    handler.send_header("Content-Type", "text/plain; charset=utf-8")
    handler.send_header("Content-Length", str(len(payload)))
    handler.end_headers()
    handler.wfile.write(payload)
    handler.wfile.flush()


def send_ok(handler, message="OK", status=200):
    """Send a successful text response."""
    send_text(handler, status, message, append_newline=False)


def send_json(handler, status, payload):
    """Send a JSON response with a stable UTF-8 content type."""
    body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
    handler.send_response(status)
    handler.send_header("Content-Type", "application/json; charset=utf-8")
    handler.send_header("Content-Length", str(len(body)))
    handler.end_headers()
    handler.wfile.write(body)
    handler.wfile.flush()


def send_error(handler, message, status=400):
    """Send a client or server error response."""
    send_text(handler, status, message)


def send_method_not_allowed(handler):
    """Send the consistent response for an unsupported HTTP method/path."""
    send_text(handler, 405, "Method Not Allowed")


def validate_timestamp(
    handler,
    request_url,
    body_text,
    query_params,
    timeout_s=20.0,
    logger=None,
):
    """Validate a seconds or milliseconds timestamp from query or JSON body."""
    timestamp = None
    values = query_params.get("timestamp")
    if values:
        timestamp = values[0]
    elif body_text:
        try:
            payload = json.loads(body_text)
            if not isinstance(payload, dict):
                send_error(handler, "JSON body must be an object")
                return False
            timestamp = payload.get("timestamp")
        except json.JSONDecodeError:
            send_error(handler, "Invalid JSON body")
            return False

    if timestamp is None:
        send_error(handler, "Missing timestamp")
        return False

    try:
        request_time = float(timestamp)
        if request_time > 100000000000:
            request_time /= 1000.0
    except (TypeError, ValueError):
        send_error(handler, "Invalid timestamp")
        return False

    age = abs(time.time() - request_time)
    if logger:
        logger(
            f"timestamp check url={request_url} "
            f"timestamp={timestamp} age={age:.3f}s"
        )
    if age > timeout_s:
        send_text(handler, 408, "Request timestamp expired")
        return False
    return True
