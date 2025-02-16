import matplotlib.pyplot as plt

def visualize(file):
    with open(file, 'r') as f:
        lines = f.readlines()
        x = []
        y = []
        for line in lines:
            x.append(float(line.split()[0]))
            y.append(float(line.split()[1]))
            ax = plt.gca()
            ax.set_xlim([0, 4])
            ax.set_ylim([0, 2])
            plt.plot(x, y)
            plt.pause(0.01)


if __name__ == '__main__':
    visualize('out')
