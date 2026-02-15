from lerobot.policy import Pi0Policy
# from lerobot.policy import SmolVLAPolicy

# Load the pretrained pi0 model
policy = Pi0Policy.from_pretrained("lerobot/pi0")

# policy = SmolVLAPolicy.from_pretrained("lerobot/smolvla_base")

# Prepare your input batch (replace with your actual data)
batch = {}  # Dictionary containing necessary inputs like image, language instruction, etc.

# Select an action based on the input batch
action = policy.select_action(batch)

# The 'action' variable now contains the model's output, which you can use to control your robot.
print(action)

