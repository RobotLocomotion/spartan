__author__ = 'manuelli'
import numpy as np
import forcespro as fp
import importlib
import get_userid
import os
import sys
NUM_FRICTION_CONE_BASIS_VECTORS = 4



class ForcesProQP:

    def __init__(self, numContactsList=[1]):
        self.numContactsList = numContactsList
        self.initializeModels(numContactsList)

    def initializeModels(self, numContactsList):
        self.modelData = {}
        for numContacts in numContactsList:
            self.initializeSingleModel(numContacts)

    def initializeSingleModel(self, numContacts):

        # struct that keeps track of all the model/solver data

        data = dict()
        # keep track of the variable names.
        # List that takes and index and gives you back the name
        varNames = []
        # idx's are always of type int
        varIdx = np.zeros((numContacts, NUM_FRICTION_CONE_BASIS_VECTORS), dtype=int)
        counter = 0 # start at 1 since forcespro is 1-indexed

        for i in xrange(0,numContacts):
            for j in xrange(0, NUM_FRICTION_CONE_BASIS_VECTORS):
                name = "alpha_%i_%i" % (i,j)
                varNames.append(name)

                varIdx[i,j] = counter
                counter += 1

        data['varNames'] = varNames
        data['varIdx'] = varIdx
        numVars = numContacts*NUM_FRICTION_CONE_BASIS_VECTORS


        # setup the forces pro problem
        # WARNING: everything in forcespro is 1-indexed, not 0-indexed like standard python
        stages = fp.MultistageProblem(1)
        stages.dims[0]['n'] = numVars  # dimension of decision variables
        stages.dims[0]['r'] = 0  # number of eq constraints
        stages.dims[0]['l'] = numVars  # number of lower bounds
        stages.dims[0]['u'] = 0  # number of upper bounds
        stages.dims[0]['p'] = 0  # number of polytopic constraints
        stages.dims[0]['q'] = 0  # number of quadratic constraints

        # quadratic cost term (will get filled in later as a parameter)
        stages.cost[0]['H'] = np.zeros((numVars, numVars))

        # linear cost term (will get filled in later as a parameter)
        stages.cost[0]['f'] = np.zeros((numVars,))

        # variable idexes that have lower bound constraints
        stages.ineq[0]['b']['lbidx'] = range(1, numVars + 1)
        stages.ineq[0]['b']['lb'] = np.zeros((numVars,)) # lb for those variables

        stages.newParam("QuadraticCost", [1], 'cost.H') # parameter for quadratic cost
        stages.newParam("LinearCost", [1], 'cost.f') # parameter for linear cost

        stages.newOutput('alpha', 1, range(1,numVars + 1)) # output capturing the solution

        # solver settings
        solverName = 'forcespro_qp_' + str(numContacts) + '_contact_point'
        data['solverName'] = solverName
        stages.codeoptions['name'] = solverName
        stages.codeoptions['printlevel'] = 0
        stages.codeoptions['optlevel'] = 0 # this is the highest level of optimization
        stages.codeoptions['overwrite'] = 1 # overwrite old solver with new solver

        # generate the actual code
        # store it in the bin directory
        pathToCPFBin = os.getenv('SPARTAN_SOURCE_DIR') + '/src/ContactParticleFilter/bin/'
        os.chdir(pathToCPFBin)
        stages.generateCode(get_userid.userid)

        sys.path.append(pathToCPFBin)
        solverModule = importlib.import_module(solverName + '_py')
        params = getattr(solverModule,solverName + "_params")
        solveMethod = getattr(solverModule, solverName + "_solve")


        # store these methods/attributes in the dict for easy access later
        data['solverModule'] = solverModule
        data['params'] = params
        data['solveMethod'] = solveMethod
        data['stages'] = stages

        self.modelData[numContacts] = data


    def solve(self, numContacts, residual, H_list, W):
        H = np.concatenate(H_list, axis=1)

        # WARNING: need to multiply this by 2
        # Forces Pro has 1/2*x'*Q*x type cost term which
        # is different from gurobi
        Q = 2.0*np.dot(np.dot(H.transpose(),W),H)

        f = -2.0*np.dot(np.dot(residual.transpose(), W),H)
        constant = np.dot(np.dot(residual.transpose(), W), residual)

        model = self.modelData[numContacts]
        params = model['params']
        solveMethod = model['solveMethod']

        params['QuadraticCost'] = Q
        params['LinearCost'] = f

        [solverout, exitflag, info] = solveMethod(params)
        alpha = solverout['alpha']

        solnData = self.parseModelSolution(solverout, exitflag, info, numContacts)

        # print "forcespro objective: ", solnData['info'].pobj
        # print "forcespro objective with constant: ", solnData['info'].pobj
        return solnData

    def parseModelSolution(self, solverout, exitflag, info, numContacts):
        d = {}
        d['alphaVals'] = {}
        alpha = solverout["alpha"]

        varIdx = self.modelData[numContacts]['varIdx']

        for i in xrange(0,numContacts):
            for j in xrange(0,4):
                idx = varIdx[i,j]
                d['alphaVals'][i,j] = alpha[idx]

        d['objectiveValue'] = info.pobj
        d['info'] = info
        d['exitflag'] = exitflag

        return d



