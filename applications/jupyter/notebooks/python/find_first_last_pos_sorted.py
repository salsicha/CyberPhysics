
# Given an array of integers nums sorted in non-decreasing order, 
# find the starting and ending position of a given target value.
# If target is not found in the array, return [-1, -1].
# You must write an algorithm with O(log n) runtime complexity.

# Example 1:
# Input: nums = [5,7,7,8,8,10], target = 8
# Output: [3,4]

# Example 2:
# Input: nums = [5,7,7,8,8,10], target = 6
# Output: [-1,-1]

# Example 3:
# Input: nums = [], target = 0
# Output: [-1,-1]

# Constraints:
#     0 <= nums.length <= 105
#     -109 <= nums[i] <= 109
#     nums is a non-decreasing array.
#     -109 <= target <= 109


from typing import List

class Solution:
    def searchRange(self, nums: List[int], target: int) -> List[int]:
        list_middle = int(len(nums) / 2)

        middle_val = nums[list_middle]

        if middle_val == target:
            target_range = []
            if nums[list_middle - 1] == target:
                target_range.append(list_middle - 1)
                target_range.append(list_middle)
            elif nums[list_middle + 1] == target:
                target_range.append(list_middle)
                target_range.append(list_middle + 1)
            else:
                target_range.append(list_middle)
                target_range.append(list_middle)
            return target_range
        if len(nums) <= 3:
            return [-1, -1]

        if middle_val > target:
            nums = nums[0:middle_val]
        if middle_val < target:
            nums = nums[middle_val:-1]
        
        return self.searchRange(nums, target)

nums = [5,7,7,8,8,10]
target = 8

sol = Solution()
print(sol.searchRange(nums, target))