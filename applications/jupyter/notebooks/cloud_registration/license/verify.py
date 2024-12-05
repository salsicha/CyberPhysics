
from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.asymmetric import padding
from cryptography.hazmat.backends import default_backend
from cryptography.hazmat.primitives import serialization
import base64
from datetime import datetime
import fingerprint
import yaml
try:
    from yaml import CLoader as Loader
except ImportError:
    from yaml import Loader



# TODO: make sure sign.py is outside deployment path, but public.pem and verify.py is not



license_file = "license.yaml"
public_key = "public.pem"

with open(license_file, "rb") as license_file:
    yaml_data = yaml.load(license_file.read(), Loader=Loader)

message_data = yaml_data[0]

signature = yaml_data[1]
signature_base64 = signature.encode('UTF-8')
signature = base64.b64decode(signature_base64)

with open(public_key, "rb") as key_file:
    public_key = serialization.load_pem_public_key(key_file.read(), backend=default_backend())

result = True

try:
    public_key.verify(
        signature,
        yaml.dump(message_data).encode('UTF-8'),
        padding.PSS(mgf = padding.MGF1(hashes.SHA256()), salt_length = padding.PSS.MAX_LENGTH),
        hashes.SHA256()
    )
    # print("verification success \n")
except Exception:
    # print("verification failed \n")
    result = False

# Verify license period
end_date = datetime.fromtimestamp(int(float(message_data['end_date'])))
now = datetime.now().timestamp()
if now > end_date.timestamp():
    result = False

if not fingerprint.verify_hash(fingerprint.compute_machine_code(), message_data['machine_id']):
    result = False

print(result)