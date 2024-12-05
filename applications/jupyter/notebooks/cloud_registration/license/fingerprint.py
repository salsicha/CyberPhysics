# Device fingerprinting

from passlib.hash import sha256_crypt

def get_dbus_machine_id():
    try:
        with open("/etc/machine-id") as f:
            return f.read().strip()
    except:
        pass
    try:
        with open("/var/lib/dbus/machine-id") as f:
            return f.read().strip()
    except:
        pass
    return ""

def get_inodes():
    import os
    files = ["/bin", "/etc", "/lib", "/root", "/sbin", "/usr", "/var"]
    inodes = []
    for file in files:
        try:
            inodes.append(os.stat(file).st_ino)
        except: 
            pass
    return "".join([str(x) for x in inodes])

def compute_machine_code():
    return get_dbus_machine_id() + get_inodes()

def get_SHA256(string):
    """
    Compute the SHA256 signature of a string.
    """
    return sha256_crypt.hash(string.encode("utf-8"))

def verify_hash(hash1, hash2):
    return sha256_crypt.verify(hash1, hash2)

def get_machine_code():
    machine_code = get_SHA256(compute_machine_code())
    return machine_code