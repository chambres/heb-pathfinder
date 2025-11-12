import http.server
import ssl
import os
from pathlib import Path

PORT = 4443
CERT_FILE = Path("selfsigned.pem")
KEY_FILE = Path("selfsigned.key")

# --- 1️⃣ Generate certs automatically if missing ---
if not CERT_FILE.exists() or not KEY_FILE.exists():
    print("Generating self-signed certificate...")
    from cryptography import x509
    from cryptography.x509.oid import NameOID
    from cryptography.hazmat.primitives import hashes, serialization
    from cryptography.hazmat.primitives.asymmetric import rsa
    import datetime

    key = rsa.generate_private_key(public_exponent=65537, key_size=2048)
    subject = issuer = x509.Name([
        x509.NameAttribute(NameOID.COUNTRY_NAME, "US"),
        x509.NameAttribute(NameOID.ORGANIZATION_NAME, "Localhost Dev"),
        x509.NameAttribute(NameOID.COMMON_NAME, "localhost"),
    ])
    cert = (
        x509.CertificateBuilder()
        .subject_name(subject)
        .issuer_name(issuer)
        .public_key(key.public_key())
        .serial_number(x509.random_serial_number())
        .not_valid_before(datetime.datetime.utcnow())
        .not_valid_after(datetime.datetime.utcnow() + datetime.timedelta(days=365))
        .add_extension(x509.SubjectAlternativeName([x509.DNSName("localhost")]), critical=False)
        .sign(key, hashes.SHA256())
    )

    with open(KEY_FILE, "wb") as f:
        f.write(
            key.private_bytes(
                encoding=serialization.Encoding.PEM,
                format=serialization.PrivateFormat.TraditionalOpenSSL,
                encryption_algorithm=serialization.NoEncryption(),
            )
        )

    with open(CERT_FILE, "wb") as f:
        f.write(cert.public_bytes(serialization.Encoding.PEM))

# --- 2️⃣ Serve HTTPS ---
server_address = ('localhost', PORT)
handler = http.server.SimpleHTTPRequestHandler
httpd = http.server.HTTPServer(server_address, handler)
httpd.socket = ssl.wrap_socket(
    httpd.socket,
    keyfile=str(KEY_FILE),
    certfile=str(CERT_FILE),
    server_side=True
)

print(f"Serving HTTPS on https://localhost:{PORT}")
httpd.serve_forever()

