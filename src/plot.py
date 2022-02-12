import matplotlib.pyplot as plt
import sys

def plot_csv(filename):
    # one set of x-values
    x = []
    # name for x-axis
    xName = ""
    # names for y-axises
    y = []
    # multiple arrays of y-values
    yNames = []
    with open(filename, "r") as file:
        data = [line.strip().split(",") for line in file.readlines()]
        # first line contains axis names
        header = data[0]

        xName = header[0]
        yNames = header[1:]

        y = [[] for i in range(len(yNames))]

        for columns in data[1:]:
            # x-value is always the first element
            x.append(float(columns[0]))
            # each y-value followed by corresponding x-value
            for i,yVal in enumerate(columns[1:]):
                y[i].append(float(yVal))

    fig, ax = plt.subplots(len(y), 1)

    fig.canvas.manager.set_window_title(filename)

    for i,yValues in enumerate(y):
        #ax[i].set_xlabel(xName)
        ax[i].set_ylabel(yNames[i])
        ax[i].plot(x, yValues)

    plt.show()


# loop through commandline arguments and
# plot provided csv files
if len(sys.argv) > 1:
    for arg in sys.argv[1:]:
        plot_csv(arg)
