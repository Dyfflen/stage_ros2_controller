import numpy as np

def calculate_corrected_target(desired_target, calibration_data):
    p_a_actual = calibration_data[0]['actual_position']
    e_a = calibration_data[0]['error_vector']
    p_b_actual = calibration_data[1]['actual_position']
    e_b = calibration_data[1]['error_vector']
    
    total_dist_ab = np.linalg.norm(p_a_actual - p_b_actual)
    dist_a_new = np.linalg.norm(p_a_actual - desired_target)
    
    if total_dist_ab == 0:
        t = 0
    else:
        t = dist_a_new / total_dist_ab
    
    t = np.clip(t, 0, 1)
    
    estimated_error = (1 - t) * e_a + t * e_b
    corrected_target = desired_target + estimated_error
    
    return corrected_target, estimated_error

data = [{'actual_position': np.array([7.00, -3.00]), 'error_vector': np.array([0.69, 0.17])},
        {'actual_position': np.array([7.00, 7.00]), 'error_vector': np.array([0.84, 0.66])}]

target_1 = np.array([7.0, -3.0]) # change the values of target 1 here
target_2 = np.array([7.0, 7.0]) # change the values of target 2 here

final_target_to_send, predicted_error = calculate_corrected_target(target_1, data)

print("Target 1:")
print(f"Desired Target: {target_1}")
print(f"Estimated Error for this point: ({predicted_error[0]:.2f}, {predicted_error[1]:.2f})")
print(f"Corrected Target to send to the robot: ({final_target_to_send[0]:.2f}, {final_target_to_send[1]:.2f})")

final_target_to_send, predicted_error = calculate_corrected_target(target_2, data)

print("\nTarget 2:")
print(f"Desired Target: {target_2}")
print(f"Estimated Error for this point: ({predicted_error[0]:.2f}, {predicted_error[1]:.2f})")
print(f"Corrected Target to send to the robot: ({final_target_to_send[0]:.2f}, {final_target_to_send[1]:.2f})")