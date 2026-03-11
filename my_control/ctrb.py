import numpy as np

def get_controllability_rank(A, B):
    """
    Computes the rank of the controllability matrix for matrices A and B.
    
    Parameters:
    A (ndarray): State matrix (n x n)
    B (ndarray): Input matrix (n x m)
    
    Returns:
    int: The rank of the controllability matrix.
    """
    n = A.shape[0]
    # Initialize the controllability matrix with B
    ctrb_matrix = B
    
    # Iteratively compute and stack [B, AB, A^2B, ..., A^(n-1)B]
    current_term = B
    for _ in range(1, n):
        current_term = A @ current_term
        ctrb_matrix = np.hstack((ctrb_matrix, current_term))
        
    # Return the rank of the resulting matrix
    return np.linalg.matrix_rank(ctrb_matrix)