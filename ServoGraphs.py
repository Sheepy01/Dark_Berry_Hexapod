import matplotlib.pyplot as plt

# Read values from the file, ignoring empty lines and non-numeric values
values = []
with open('TibiaAngles.txt', 'r') as file:
    for line_num, line in enumerate(file, start=1):
        line = line.strip()
        if line:
            try:
                value = float(line)
                values.append(value)
            except ValueError:
                print(f"Skipping non-numeric value on line {line_num}: {line}")

# Create a list of line numbers as the x-axis (assuming one line per time point)
time_points = list(range(1, len(values) + 1))

# Plot the values against time
plt.plot(time_points, values, marker='o', linestyle='-', color='b')

# Add labels and title
plt.xlabel('Time')
plt.ylabel('Angles')
plt.title('Angles Over Time')

# Show the plot
plt.show()