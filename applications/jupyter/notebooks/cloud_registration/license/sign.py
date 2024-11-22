
from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.asymmetric import padding
from cryptography.hazmat.backends import default_backend
from cryptography.hazmat.primitives import serialization

import fingerprint
import base64
from datetime import datetime
import subprocess
import yaml
try:
    from yaml import CLoader as Loader
except ImportError:
    from yaml import Loader


one_pass = "./one_pass.sh"


# admin_pass = input("Enter private key password: ")
admin_pass = "Symbio1234"

# machine_code = input("Enter machine ID: ")
machine_code = fingerprint.get_SHA256(fingerprint.compute_machine_code())

# email_addr = input("Enter your 1Password email address: ")
email_addr = "alex.moran@symb.io"

# expiration_date = input("Enter license expiration date (year, month, day): ")
expiration_date = "2023, 1, 1"

ex = expiration_date.split(",")
end_date = datetime(int(float(ex[0])), int(float(ex[1])), int(float(ex[2]))).timestamp()

message_data = {"end_date": str(end_date), "machine_id": machine_code}

priv_key = subprocess.check_output([one_pass, email_addr], shell = True)

private_key = serialization.load_pem_private_key(
    priv_key, password=admin_pass.encode('UTF-8'), backend=default_backend()
)

signature = private_key.sign(
    yaml.dump(message_data).encode('UTF-8'),
    padding.PSS(mgf = padding.MGF1(hashes.SHA256()), salt_length = padding.PSS.MAX_LENGTH),
    hashes.SHA256()
)
signature_base64 = base64.b64encode(signature)

cert = [message_data, signature_base64.decode("utf-8")]
with open('license.yaml', 'w+') as outfile:
    yaml.dump(cert, outfile, default_flow_style=False)

print("Now email license.yaml to the SI")



# TODO: make sure sign.py is outside deployment path, but public.pem is in it

# TODO: what pip packages need to be installed? pyyaml, passlib

# TODO: add private.pem to gitignore
# TODO: add license.yml to gitignore

# TODO: make README
# README:
# 1. SI script prints machine id
# 2. machine id in google form
# 3. symbiot gets machine id from google spreadsheet
# 4. symbiot gets private.pem from 1Password
# 5. symbiot runs symbio-dcs/licensing/sign.py that takes key and machine id as args and product ID, returns license.yml
# 6. symbiot sends license.yml to SI

# TODO: private.pem vault and timestamp
# TODO: private.pem in 1Password must have Symbio-DCS version associated with it
# TODO: make script that reads/writes 1Password private.pem use git hash in file name

# TODO: product ID management?
# SI selects product in form
# Symbiot takes name and uses appropriate code
# puts code in sign.py
# gets license.yml
# emails license.yml
# TODO: app needs config with product ID
# verify.py needs to read config and check that product ID in config matches license
# the product ID can be sent to adminapi, and adminapi can then prevent apps that dont match that name
# so then start_app("reference") would only work for the "reference" product
# there probably needs to be a table in the database that maps product names to product IDs
# TODO: product ID -> product name in spreadsheet?
# then product IDs in database too






# argumentList = sys.argv[1:]
# arguments, values = getopt.getopt(sys.argv, "pm:")
# print("args: ", arguments)
# print("values: ", values)
