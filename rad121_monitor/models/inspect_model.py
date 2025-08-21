import pickle
import numpy as np

model_path = '/home/brendoncintas/radmon_ws/install/rad121_monitor/share/rad121_monitor/models/gp_model.pkl'

print(f"\n--- Deep inspection of: {model_path} ---\n")

try:
    with open(model_path, 'rb') as f:
        data = pickle.load(f)

    if isinstance(data, dict):
        print("File contains a dictionary. Inspecting each key:\n")
        for key, value in data.items():
            print(f"Key: '{key}'")
            print(f"  - Type: {type(value)}")

            # Try to convert to numpy array to check shape
            try:
                value_np = np.array(value)
                print(f"  - Shape as array: {value_np.shape}")
            except:
                print("  - Could not determine shape.")

            # Print the value itself
            print(f"  - Value: {value}")
            print("-" * 20)
    else:
        print(f"File does not contain a dictionary. It contains an object of type: {type(data)}")

except Exception as e:
    print(f"An error occurred: {e}")