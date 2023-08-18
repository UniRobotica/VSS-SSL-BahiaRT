def calculate_linear_speed(positions: list[float], fps=59.):
        
        size = len(positions)
        if size <= 1:
            return 0
        
        positions_diff = []
        for e in range(1, size):
            if positions[e] - positions[e-1] < 1:
                positions_diff.append(positions[e] - positions[e-1])
        
        speed = sum(positions_diff) / len(positions_diff) / (60/fps)
        
        if abs(speed) < 1:
            return speed
        else:
            return 0