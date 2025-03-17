import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import PercentFormatter

# Data for Grouped Bar Plot
data = np.array([[1.0, 0.0], [1.0, 1.0], [1.0, 0.4], [0.7, 0.1]])

# Create Grouped Bar Plot
fig, ax = plt.subplots()
bar_width = 0.35
index = np.arange(data.shape[0])

bar1 = ax.bar(index, data[:, 0], bar_width, label='With downwash algorithm')
bar2 = ax.bar(index + bar_width, data[:, 1], bar_width, label='Without downwash algorithm')

# Add success rates above each bar as percentages
for i, bar in enumerate(bar1):
    ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.02, 
            f'{data[i, 0]:.0%}', ha='center', va='bottom', fontsize=9)

for i, bar in enumerate(bar2):
    ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.02, 
            f'{data[i, 1]:.0%}', ha='center', va='bottom', fontsize=9)

# Customize Group Names
groupNames = ['Light bulb Installation', 'Drawer Opening', 'Closet Opening', 'Pipe Inspection']
ax.set_xticks(index + bar_width / 2)  # Set x-ticks to be in the center of the bars
ax.set_xticklabels(groupNames)         # Apply custom group names

# Customize Axes
ax.spines['top'].set_visible(False)    # Hide top axis
ax.spines['right'].set_visible(False)  # Hide right axis
ax.spines['left'].set_linewidth(2)    # Make left axis thicker
ax.spines['bottom'].set_linewidth(2)  # Make bottom axis thicker
ax.tick_params(axis='x', direction='out')  # Set x-axis ticks to point outward
ax.tick_params(axis='y', direction='out')  # Set y-axis ticks to point outward

# Set y-axis to display percentages
ax.yaxis.set_major_formatter(PercentFormatter(xmax=1.0))

xmin, xmax = ax.get_xlim() 
ymin, ymax = ax.get_ylim()
 
# Removing the default axis on all sides:
for side in ['bottom','right','top','left']:
    ax.spines[side].set_visible(False)

ax.xaxis.set_ticks_position('none') # Tick markers
ax.yaxis.set_ticks_position('none')
 
# Wider figure for demonstration
fig.set_size_inches(4, 2.2)
 
# Get width and height of axes object to compute 
# matching arrowhead length and width
dps = fig.dpi_scale_trans.inverted()
bbox = ax.get_window_extent().transformed(dps)
width, height = bbox.width, bbox.height
 
# Manual arrowhead width and length
hw = 1./30.*(ymax-ymin) 
hl = 1./30.*(xmax-xmin)
lw = 1. # Axis line width
ohg = 0.3 # Arrow overhang
 
# Compute matching arrowhead length and width
yhw = hw/(ymax-ymin)*(xmax-xmin)* height/width 
yhl = hl/(xmax-xmin)*(ymax-ymin)* width/height
 
# Draw x and y axis
ax.arrow(xmin, 0, xmax-xmin, 0., fc='k', ec='k', lw=lw, 
         head_width=hw, head_length=hl, overhang=ohg, 
         length_includes_head=True, clip_on=False) 
 
ax.arrow(xmin, ymin, 0., ymax-ymin, fc='k', ec='k', lw=lw, 
         head_width=yhw, head_length=yhl, overhang=ohg, 
         length_includes_head=True, clip_on=False) 

# Add Labels and Title
# ax.set_xlabel('Groups')
ax.set_ylabel('Success rate')
# ax.set_title('Grouped Bar Plot with Axis Arrows')

# Add Legend
ax.legend()

# Show the plot
plt.show()
