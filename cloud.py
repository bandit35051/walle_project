import os
from google.cloud import storage
from google.oauth2 import service_account

# Define directories relative to this file
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_DIR = os.path.join(BASE_DIR, "config")
DATA_DIR = os.path.join(BASE_DIR, "data")

# Updated paths using the config and data directories
CREDENTIALS_PATH = os.path.join(CONFIG_DIR, "wall_e_key.json")
BUCKET_NAME = "wall-e-memory-bucket"
MEMORY_FILE = os.path.join(DATA_DIR, "wall_e_memory.json")

def init_cloud():
    try:
        credentials = service_account.Credentials.from_service_account_file(CREDENTIALS_PATH)
        storage_client = storage.Client(credentials=credentials)
        bucket = storage_client.bucket(BUCKET_NAME)
        print("Connected to Google Cloud successfully.")
        return bucket
    except Exception as e:
        print(f"Failed to connect to Google Cloud: {e}")
        return None

def cloud_backup(bucket):
    try:
        blob = bucket.blob("wall_e_memory.json")
        blob.upload_from_filename(MEMORY_FILE)
        print("Memory backup completed to cloud.")
    except Exception as e:
        print(f"Failed to back up memory to cloud: {e}")

def check_for_updates(bucket):
    try:
        blob = bucket.blob("Wall_E_Pi_Upgrade.py")
        if blob.exists():
            new_upgrade_file = os.path.join(BASE_DIR, "Wall_E_Pi_Upgrade_New.py")
            blob.download_to_filename(new_upgrade_file)
            print("New software update detected. Preparing to update.")
            os.system(f"sudo mv {new_upgrade_file} Wall_E_Pi_Upgrade.py && sudo reboot")
        else:
            print("No updates found.")
    except Exception as e:
        print(f"Failed to check for updates: {e}")
