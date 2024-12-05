
# Given an integer array nums, 
# return all the triplets [nums[i], nums[j], nums[k]] 
# such that i != j, i != k, and j != k, and nums[i] + nums[j] + nums[k] == 0.

# Notice that the solution set must not contain duplicate triplets.

# Input: nums = [-1,0,1,2,-1,-4]
# Output: [[-1,-1,2],[-1,0,1]]

# Input: nums = []
# Output: []

# Input: nums = [0]
# Output: []


nums = [-1,0,1,2,-1,-4]

output = []
key_dict = []

for i, x in enumerate(nums):
    for j, y in enumerate(nums):
        for k, z in enumerate(nums):
            if len([i, j, k]) == len(set([i, j, k])):
                if x + y + z == 0:

                    success = True
                    for dict in key_dict:
                        if x in dict and y in dict and z in dict:
                            success = False

                    if success:
                        key_dict.append({x: 1, y: 1, z: 1})
                        output.append([x, y, z])

print(output)

