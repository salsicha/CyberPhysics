
# Given two sorted arrays nums1 and nums2 of size m and n respectively, return the median of the two sorted arrays.
# The overall run time complexity should be O(log (m+n)).

# Example 1:
# Input: nums1 = [1,3], nums2 = [2]
# Output: 2.00000
# Explanation: merged array = [1,2,3] and median is 2.

# Example 2:
# Input: nums1 = [1,2], nums2 = [3,4]
# Output: 2.50000
# Explanation: merged array = [1,2,3,4] and median is (2 + 3) / 2 = 2.5.

# Constraints:
#     nums1.length == m
#     nums2.length == n
#     0 <= m <= 1000
#     0 <= n <= 1000
#     1 <= m + n <= 2000
#     -106 <= nums1[i], nums2[i] <= 106


# Logs example 1:
# 10^2 = 100
# log_{10} 100 = 2
# Example 2:
# 10^1.69897000434 = 50
# log_{10} 50 = 1.69897000434

# Binary search
# If the array’s length is 8, then the binary search loop will execute 3 times because log2(8) = 3


from heapq import merge

nums1 = [1,2], 
nums2 = [3,4]

nums_both = []
i = 0
j = 0

m = len(nums1)
n = len(nums2)

# TODO:
# this makes the complexity O(m+n)
# but it needs to be O(log(m+n))
# O(log(x)) requires a binary search

while i < m and j < n:
    if nums1[i] <= nums2[j]:
        nums_both.append(nums1[i])
        i += 1
    else:
        nums_both.append(nums2[j])
        j += 1

total_length = m + n
median_len = int(total_length / 2)

if total_length == (median_len * 2):
    pass
