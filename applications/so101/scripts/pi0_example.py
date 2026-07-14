import torch

from lerobot.policies.pi0.modeling_pi0 import PI0Policy
# from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy

device = "cuda" if torch.cuda.is_available() else "cpu"

# Load the pretrained pi0 model
policy = PI0Policy.from_pretrained("lerobot/pi0")
# policy = SmolVLAPolicy.from_pretrained("lerobot/smolvla_base")
policy.to(device)
policy.eval()

# Placeholder batch shaped from the checkpoint's input features (camera
# images, robot state) plus a language instruction; replace with real
# robot observations.
batch = {
    name: torch.zeros(1, *feature.shape, device=device)
    for name, feature in policy.config.input_features.items()
}
batch["task"] = ["pick up the cube"]

# Select an action based on the input batch
with torch.inference_mode():
    action = policy.select_action(batch)

# The 'action' variable now contains the model's output, which you can use to control your robot.
print(action)
