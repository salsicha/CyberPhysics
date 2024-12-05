
# https://leetcode.com/problems/search-in-rotated-sorted-array/

# There is an integer array nums sorted in ascending order (with distinct values).
# Prior to being passed to your function, 
# nums is possibly rotated at an unknown pivot index k (1 <= k < nums.length) 
# such that the resulting array is [nums[k], nums[k+1], ..., nums[n-1], nums[0], nums[1], ..., nums[k-1]] (0-indexed). For example, [0,1,2,4,5,6,7] might be rotated at pivot index 3 and become [4,5,6,7,0,1,2].
# Given the array nums after the possible rotation and an integer target, return the index of target if it is in nums, or -1 if it is not in nums.
# You must write an algorithm with O(log n) runtime complexity.

# Example 1:
# Input: nums = [4,5,6,7,0,1,2], target = 0
# Output: 4

# Example 2:
# Input: nums = [4,5,6,7,0,1,2], target = 3
# Output: -1

# Example 3:
# Input: nums = [1], target = 0
# Output: -1


# Constraints:
#     1 <= nums.length <= 5000
#     -10^4 <= nums[i] <= 10^4
#     All values of nums are unique.
#     nums is an ascending array that is possibly rotated.
#     -10^4 <= target <= 10^4



from distutils.command.build_scripts import first_line_re


class Solution:
    def search(self, nums: list[int], target: int) -> int:

        index = 0

        while True:
            print(nums)

            arr_median = int(len(nums) / 2)

            if target == nums[0]:
                break
            elif target == nums[-1]:
                index += len(nums) - 1
                break
            elif target == nums[arr_median]:
                index += arr_median
                break
            elif len(nums) <= 3:
                index = -1
                break

            if nums[arr_median] < nums[-1] and nums[arr_median] < nums[0]:
                index += 0
                nums = nums[0:arr_median]

            elif nums[arr_median] > nums[-1] and nums[arr_median] > nums[0]:
                index += arr_median
                nums = nums[arr_median:-1]

            elif nums[arr_median] > nums[-1] and nums[arr_median] < nums[0]:
                index += 0
                nums = nums[0:arr_median]

        return index


nums = [1]

# nums = [7,0,1,2,4,5,6] # L R M
# nums = [5,6,7,0,1,2,4] # L R M

# nums = [2,4,5,6,7,0,1] # M L R
# nums = [1,2,4,5,6,7,0] # M L R

# nums = [0,1,2,4,5,6,7] # R M L

target = 0

sol = Solution()
ret = sol.search(nums, target)

print("ret: ", ret)
