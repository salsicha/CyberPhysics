
def isSafe(i, j, visited, grid):
    rows = len(grid)
    cols = len(grid[0])
    return (i >= 0 and i < rows and
            j >= 0 and j < cols and
            not visited[i][j] and grid[i][j])

def DFS(i, j, visited, grid):
    row_pos = [1, 0, 0, -1]
    col_pos = [0, -1, 1, 0]
        
    visited[i][j] = True

    for k in range(len(row_pos)):
        if isSafe(i + row_pos[k], j + col_pos[k], visited, grid):
            DFS(i + row_pos[k], j + col_pos[k], visited, grid)

def countIslands(grid):

    rows = len(grid)
    cols = len(grid[0])

    visited = [[False for c in range(cols)] for r in range(rows)]

    count = 0
    for i in range(rows):
        for j in range(cols):
            if visited[i][j] == False and grid[i][j] == 1:
                DFS(i, j, visited, grid)
                count += 1

    return count

 
graph = [[1, 1, 0, 0, 0],
         [0, 1, 0, 0, 1],
         [1, 0, 0, 1, 1],
         [0, 0, 0, 0, 0],
         [1, 0, 1, 0, 1]]

print ("Number of islands is:")
print (countIslands(graph))