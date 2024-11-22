
# Given an array nums of n integers, 
# return an array of all the unique quadruplets 
# [nums[a], nums[b], nums[c], nums[d]] such that:

# 0 <= a, b, c, d < n
# a, b, c, and d are distinct.
# nums[a] + nums[b] + nums[c] + nums[d] == target
# You may return the answer in any order.

 
# Example 1:
# Input: nums = [1,0,-1,0,-2,2], target = 0
# Output: [[-2,-1,1,2], [-2,0,0,2], [-1,0,0,1]]

# Example 2:
# Input: nums = [2,2,2,2,2], target = 8
# Output: [[2,2,2,2]]

output = []

nums = [1,0,-1,0,-2,2]
target = 0

for i, a in enumerate(nums):
    for j, b in enumerate(nums):
        if j > i:
            for k, c in enumerate(nums):
                if k > j:
                    for l, d in enumerate(nums):
                        if l > k:
                            index_list = [i, j, k, l]
                            if len(index_list) == len(set(index_list)):
                                if a + b + c + d == target:
                                    output.append([a, b, c, d])

print(output)
