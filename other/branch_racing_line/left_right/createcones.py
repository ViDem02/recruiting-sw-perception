import matplotlib.pyplot as plt

# Instructions
print("🟡 Click on the plot to add cones.")
print("🔵 Press the right mouse button or 'Enter' key when finished.\n")

# Create empty list for points
points = []

# Mouse click handler
def onclick(event):
    if event.button == 1 and event.xdata is not None and event.ydata is not None:
        # Left click: add a cone
        points.append([event.xdata, event.ydata])
        plt.plot(event.xdata, event.ydata, 'yo', markersize=8)
        plt.draw()
    elif event.button == 3:  # Right click: finish
        plt.close()

# Key press handler (Enter to finish)
def onkey(event):
    if event.key == 'enter':
        plt.close()

# Set up figure
fig, ax = plt.subplots()
ax.set_title("Left click = add cone, Right click or Enter = finish")
ax.set_xlim(-5, 5)
ax.set_ylim(-5, 5)
ax.grid(True)
ax.set_aspect('equal', adjustable='box')

# Connect events
cid1 = fig.canvas.mpl_connect('button_press_event', onclick)
cid2 = fig.canvas.mpl_connect('key_press_event', onkey)

# Show window and wait for clicks
plt.show()

# Print results in MATLAB format
if points:
    print("\n✅ MATLAB Matrix:")
    print("Cones = [")
    for p in points:
        print(f"    {p[0]:.3f}    {p[1]:.3f}")
    print("];")

    # Optionally, save to file
    save = input("\nSave to 'cones.txt'? (y/n): ").strip().lower()
    if save == 'y':
        with open("cones.txt", "w") as f:
            f.write("Cones = [\n")
            for p in points:
                f.write(f"    {p[0]:.3f}    {p[1]:.3f}\n")
            f.write("];\n")
        print("✅ Saved as cones.txt")
else:
    print("No points selected.")
