import numpy as np
import matplotlib.pyplot as plt


data = [[ 2.15,  3.31,   8.29],
        [  2.65,  3.08,  3.32],
        [ 1.90,  1.94,  5.71]]

columns = ('Jacobian Odo + Reaction Forces', 'Jacobian Odometry', 'Planar Odometry')
rows = ('SandField', 'TestTrack','Motocross')

values = np.arange(0, 20, 1)
value_increment = 1

# Get some pastel shades for the colors
colors = plt.cm.bwr(np.linspace(0, 0.5, len(columns)))
n_rows = len(data)

index = np.arange(len(columns)) + 0.3
bar_width = 0.4

# Initialize the vertical-offset for the stacked bar chart.
y_offset = np.array([0.0] * len(columns))

# Plot bars and create text labels for the table
cell_text = []
for row in range(n_rows):
    plt.bar(index, data[row], bar_width, bottom=y_offset, color=colors[row])
    y_offset = y_offset + data[row]
    cell_text.append(data[row])
# Reverse colors and text labels to display the last value at the top.
colors = colors[::-1]
cell_text.reverse()

# Add a table at the bottom of the axes
the_table = plt.table(cellText=cell_text,
                      rowLabels=rows,
                      rowColours=colors,
                      colLabels=columns,
                      loc='bottom')
the_table.set_fontsize(30)
#the_table.scale(1.2, 1.2)

# Adjust layout to make room for the table:
plt.subplots_adjust(left=0.2, bottom=0.2)

plt.ylabel(r'Error [%]', fontweight='bold')
plt.yticks(values * value_increment, ['%d' % val for val in values], fontweight='bold')
plt.xticks([])
#plt.title('Error Distance Travelled')


plt.show(block=False)
plt.savefig('figures/error_distance.png', format='png')

