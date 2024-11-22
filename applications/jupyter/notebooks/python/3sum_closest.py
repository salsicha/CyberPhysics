
# Given an integer array nums of length n and an integer target, 
# find three integers in nums such that the sum is closest to target.

# Return the sum of the three integers.

# You may assume that each input would have exactly one solution.


# Example 1:
# Input: nums = [-1,2,1,-4], target = 1
# Output: 2
# Explanation: The sum that is closest to the target is 2. (-1 + 2 + 1 = 2).

# Example 2:
# Input: nums = [0,0,0], target = 1
# Output: 0


# nums = [-1, 2, 1, -4]
# target = 1

nums = [1, 2, 3, 4, -5]
target = 10

min = 10
output = 0

for i, x in enumerate(nums):
    for j, y in enumerate(nums):
        for k, z in enumerate(nums):
            index_list = [i, j, k]
            if len(index_list) == len(set(index_list)):
                abs_min = abs(x + y + z - target)

                if abs_min < min:
                    min = abs(x + y + z - target)
                    output = abs(x + y + z)

print(output)
