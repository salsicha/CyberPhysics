#!/bin/python3

import math
import os
import random
import re
import sys

#
# Complete the 'searchSuggestions' function below.
#
# The function is expected to return a 2D_STRING_ARRAY.
# The function accepts following parameters:
#  1. STRING_ARRAY repository
#  2. STRING customerQuery
#

def searchSuggestions(repository, customerQuery):
    # Write your code here
    output = []
    for i in range(len(customerQuery)):
        output_sub = []
        position = i + 1
        print("repository: ", repository[::-1])
        if position > 1:
            for word in repository[::-1]:
                word = word.lower()
                query = customerQuery[:position].lower()
                index = word.find(query)
                print("i, word, query, index: ", i, " ", word, " ", query, " ", index)
                if index == 0:
                    output_sub.append(word.lower())
            output_sub.sort()
            output.append(output_sub[:3])
    return output

    
if __name__ == '__main__':
    customerQuery = "coddle"
    repository = ['codes', 'coddles', 'coddle', 'codePhone', 'code']

    result = searchSuggestions(repository, customerQuery)

    print('\n'.join([' '.join(x) for x in result]))
    print('\n')

    # Expected output:
    # coddle coddles code
    # coddle coddles code
    # coddle coddles
    # coddle coddles
    # coddle coddles
