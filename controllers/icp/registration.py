''' 
                                    *********************
                                    *      ICP-SVD      *
                                    *                   *
                                    *                   *
                                    *      Kyriakos     *
                                    *      Lite         *
                                    *                   *
                                    *********************
Functions:

cmdparser():
• --fix, filename of fixed point point cloud. Default = 'point_cloud_a.txt'
• --mov, filename of moving point point cloud. Default = 'point_cloud_b.txt'
• --thres, error threshold. Default = 0.0001
• --iter, maximum Iterations. Default = 100
• --errplt, Boolean value that controls the generation of the error convergence to “0”. Default = True
• --plt, Boolean value that controls the generation of images of the cloud points alignment during execution.
The pictures are saved at the working directory. Default = False ~~~SLOW~~~

pointparser(filename): a function that parses the 3D points from a text files and returns a np.array of dimension (N,3)

indxtMean(index,arrays): calculates the centroid of the corresponded points in the fixed point-set.

Indxtfixed(index,arrays): returns an array of the corresponding from the fixed point cloud.

plotter(fixed,moving,i): used only when the user wants to save the images of the two point-clouds aligning to one another.
    It is set by default as False dude to the computational deficiency.

errplotter(errStorage): plots the error evolvement at the end of execution.

ICPSVD(fixed,moving,thres,maxIter,pltornot): returns the final homogeneous matrix that describes the necessary Roto – Translation
    to align the moving points with the fixed ones, a vector containing the error convergence and the transformed moving points.


Mofidied by Gonçalo Leão
Modifications: 2D visualization flag, type annotations for easier reading

'''

import numpy as np
import argparse
from timeit import default_timer as timer
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial import cKDTree as KDTree


def pointParser(fileName: str) -> np.ndarray:
    temp = []
    with open(fileName) as f:
        for l in f:
            x, y, z = l.split()
            temp.append([float(x), float(y), float(z)])
    return np.asarray(temp)


def indxtMean(index, arrays):
    indxSum = np.array([0.0, 0.0, 0.0])
    for i in range(np.size(index, 0)):
        indxSum = np.add(indxSum, np.array(arrays[index[i]]), out=indxSum, casting='unsafe')
    return indxSum / np.size(index, 0)


def indxtfixed(index, arrays):
    T = []
    for i in index:
        T.append(arrays[i])
    return np.asanyarray(T)


def plotter(fixed, moving, i, is3D: bool) -> None:
    fig = plt.figure()

    if is3D:
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(fixed[0:, 0], fixed[0:, 1], fixed[0:, 2], c='r', marker='.')
        ax.scatter(moving[0:, 0], moving[0:, 1], moving[0:, 2], c='b', marker='.')
        ax.set_xlabel('X Label')
        ax.set_ylabel('Y Label')
        ax.set_zlabel('Z Label')
    else:
        ax = fig.add_subplot(111)
        ax.scatter(fixed[0:, 0], fixed[0:, 1], c='r', marker='.')
        ax.scatter(moving[0:, 0], moving[0:, 1], c='b', marker='.')
        ax.set_xlabel('X Label')
        ax.set_ylabel('Y Label')

    # filename = str(i) + 'plot.png'
    # fig.savefig(filename)
    # plt.close(fig)
    plt.show()
    return


def errplotter(errStorage: [float]):
    plt.plot(errStorage)
    plt.ylabel('Error Convergence')
    plt.xlabel('Iterations')
    plt.show()


def ICPSVD(fixed, moving, thres: float, maxIter: int, pltornot: bool, erpltornot: bool, is3D: bool = True, verbose: bool = False) -> (np.ndarray, [float], np.ndarray):
    finhom: np.ndarray = np.identity(4)
    reqR: np.ndarray = np.identity(3)
    reqT = [0.0, 0.0, 0.0]
    TREE = KDTree(fixed)
    n = np.size(moving, 0)
    err: float = 999999
    errStorage = []
    for i in range(maxIter):
        preverr: float = err
        """Conduct a tree search"""
        distance, index = TREE.query(moving)
        """Calculate and store the Error"""
        err = np.mean(distance ** 2)
        errStorage.append(err)
        """Calculate the Centroid of moving and fixed point clouds (Corresponded points)"""
        com = np.mean(moving, 0)
        cof = indxtMean(index, fixed)
        """Form the W matrix to calculate the necessary Rot Matrix"""
        W = np.dot(np.transpose(moving), indxtfixed(index, fixed)) - n * np.outer(com, cof)
        U, _, V = np.linalg.svd(W, full_matrices=False)
        tempR = np.dot(V.T, U.T)
        """Calculate the Needed Translation"""
        tempT = cof - np.dot(tempR, com)
        """Apply the Computed Rotation and Translation to the Moving Points"""
        moving = (tempR.dot(moving.T)).T
        moving = np.add(moving, tempT)
        """Store the RotoTranslation"""
        reqR = np.dot(tempR, reqR)
        reqT = np.add(np.dot(tempR, reqT), tempT)
        if verbose:
            print('{} Cycle the MSE is equal to {}'.format(i + 1, err))
        if pltornot:
            plotter(fixed, moving, i, is3D)
            """Error Check """
        if abs(preverr - err) < thres:
            """Create a Homogeneous Matrix of the Results and plot"""
            finhom[0:3, 0:3] = reqR[0:, 0:]
            finhom[0:3, 3] = reqT[:]
            if verbose:
                print('\nThe Algorithm has exited on the {}th iteration with Error: {}\n'.format(i + 1, err))
                print('The Homogeneous Transformation matrix =\n \n {}'.format(finhom))
            break

    '''Plot Error OR Not???'''
    if erpltornot:
        errplotter(errStorage)

    return finhom, errStorage, moving


def cmdparser():
    parser = argparse.ArgumentParser()

    parser.add_argument("--fix", type=str, default='point_cloud_a.txt',
                        help=" filename of fixed point point cloud")
    parser.add_argument("--mov", type=str, default='point_cloud_b.txt',
                        help=" filename of moving point point cloud")
    parser.add_argument("--thres", type=float, default=0.0001,
                        help=" error threshold default = 0.0001")
    parser.add_argument("--iter", type=int, default=100,
                        help=" Maximum Iterations default = 100")
    parser.add_argument("--plt", type=bool, default=False,
                        help=" Generate images of convergance. ~~~SLOW~~~")
    parser.add_argument("--erplt", type=bool, default=True,
                        help="Generate graph with the Error Convergence")
    return parser.parse_args()


def main():
    # args = cmdparser()
    '''Load the 2 point clouds'''
    fixed: np.ndarray = pointParser('point_cloud_a.txt')  # pointParser(args.fix)
    moving: np.ndarray = pointParser('point_cloud_b.txt')  # pointParser(args.mov)
    '''Obtain the parsed arguments'''
    thres: float = 0.0001  # args.thres
    maxIter: int = 100  # args.iter
    pltornot: bool = True  # args.plt
    erpltornot: bool = True  # args.erplt

    print('Starting... ')
    start = timer()
    _, errStorage, _ = ICPSVD(fixed, moving, thres, maxIter, pltornot, erpltornot, is3D=True, verbose=True)
    fin = timer()
    print('\nDONE...')
    print('Total Time: {} s'.format(((fin - start))))


if __name__ == "__main__":
    main()
