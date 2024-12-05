
# Given a linked list, swap every two adjacent nodes and return its head. 
# You must solve the problem without modifying the values in the list's nodes 
# (i.e., only nodes themselves may be changed.)

# Input: head = [1,2,3,4]
# Output: [2,1,4,3]

# Input: head = []
# Output: []

# Input: head = [1]
# Output: [1]


output = []

head = [1,2,3,4,5,6]

head_1 = head[::2]
head_2 = head[1::2]

for i, x in enumerate(head_1):    
    output.append(head_2[i])
    output.append(head_1[i])

print(output)
