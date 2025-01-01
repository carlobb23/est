import math
import gurobipy as gu


class MasterProblem:
    def __init__(self, df=None, Demand=None, max_iteration=None, current_iteration=None, last=None, nr=None, start=None,
                 existing_model=None):
        if existing_model:
            # Initialize from existing model
            self.model = existing_model
            # Extract data from model
            self.nurses = list(set([int(c.ConstrName.split('(')[1].split(')')[0]) for c in self.model.getConstrs() if
                                  'lmb' in c.ConstrName]))
            print(len(self.nurses))

            self.days = list(set([int(c.ConstrName.split(',')[0].split('(')[1]) for c in self.model.getConstrs() if
                                  'demand' in c.ConstrName]))
            self.shifts = list(set([int(c.ConstrName.split(',')[1].split(')')[0]) for c in self.model.getConstrs() if
                                    'demand' in c.ConstrName]))
            self.max_iteration = max_iteration or len(self.nurses)
            self.iteration = current_iteration or 1
            self.roster = [i for i in range(1, self.max_iteration + 2)]
            self.rosterinitial = [i for i in range(1, 2)]
            self.last_itr = last
            self.max_itr = max_iteration
            self.output_len = nr or 80
            self.start = start

            # Get existing constraints
            self.cons_demand = {}
            self.cons_lmbda = {}
            self.u = self.model.addVars(self.days, self.shifts, vtype=gu.GRB.CONTINUOUS, lb=0, name='u')
            self.lmbda = self.model.addVars(self.nurses, self.roster, vtype=gu.GRB.BINARY, name='lmbda')
            for c in self.model.getConstrs():
                print(c)
                if 'demand' in c.ConstrName:
                    t, s = map(int, c.ConstrName.split('(')[1].split(')')[0].split(','))
                    self.cons_demand[t, s] = c
                elif 'lmb' in c.ConstrName:
                    i = int(c.ConstrName.split('(')[1].split(')')[0])
                    self.cons_lmbda[i] = c
                else:
                    break

        else:
            # Original initialization
            self.iteration = current_iteration
            self.max_iteration = max_iteration
            self.nurses = df['I'].dropna().astype(int).unique().tolist()
            self.days = df['T'].dropna().astype(int).unique().tolist()
            self.shifts = df['K'].dropna().astype(int).unique().tolist()
            self._current_iteration = current_iteration
            self.roster = [i for i in range(1, self.max_iteration + 2)]
            self.demand = Demand
            self.model = gu.Model("MasterProblem")
            self.cons_lmbda = {}
            self.cons_demand = {}
            self.last_itr = last
            self.max_itr = max_iteration
            self.output_len = nr
            self.demand_values = [self.demand[key] for key in self.demand.keys()]
            self.start = start

    def buildModel(self):
        self.generateVariables()
        self.generateConstraints()
        self.model.update()
        self.generateObjective()
        self.model.update()

    def generateVariables(self):
        self.u = self.model.addVars(self.days, self.shifts, vtype=gu.GRB.CONTINUOUS, lb=0, name='u')
        self.lmbda = self.model.addVars(self.nurses, self.roster, vtype=gu.GRB.BINARY, name='lmbda')

    def generateConstraints(self):
        for i in self.nurses:
            self.cons_lmbda[i] = self.model.addLConstr(1 == gu.quicksum(self.lmbda[i, r] for r in self.roster),
                                                       name="lmb(" + str(i) + ")")
        for t in self.days:
            for s in self.shifts:
                self.cons_demand[t, s] = self.model.addConstr(
                    gu.quicksum(self.lmbda[i, r] for i in self.nurses for r in self.roster[1:]) +
                    self.u[t, s] >= self.demand[t, s], "demand(" + str(t) + "," + str(s) + ")")
        return self.cons_lmbda, self.cons_demand

    def generateObjective(self):
        self.model.setObjective(gu.quicksum(self.u[t, s] for t in self.days for s in self.shifts),
                                sense=gu.GRB.MINIMIZE)

    def initCoeffs(self):
        for i in self.nurses:
            for t in self.days:
                for s in self.shifts:
                    for r in self.roster:
                        self.model.chgCoeff(self.cons_demand[t, s], self.lmbda[i, r], 0)
        self.model.update()

    def getDuals(self):
        status_dict = {
            1: "LOADED",
            2: "OPTIMAL",
            3: "INFEASIBLE",
            4: "INF_OR_UNBD",
            5: "UNBOUNDED",
            6: "CUTOFF",
            7: "ITERATION_LIMIT",
            8: "NODE_LIMIT",
            9: "TIME_LIMIT",
            10: "SOLUTION_LIMIT",
            11: "INTERRUPTED",
            12: "NUMERIC",
            13: "SUBOPTIMAL",
            14: "INPROGRESS",
            15: "USER_OBJ_LIMIT"
        }

        if self.model.status != gu.GRB.OPTIMAL:
            raise Exception(f"Status meaning: {status_dict.get(self.model.status, 'Unknown status')}")

        constraints = self.model.getConstrs()
        dual_branch = constraints[-1].Pi


        return {(d, s): self.cons_demand[d, s].Pi for d in self.days for s in self.shifts}, {(i): self.cons_lmbda[i].Pi for i in self.nurses}, dual_branch

    def updateModel(self):
        self.model.update()

    def startSol(self, start_vals):
        for i in self.nurses:
            for t in self.days:
                for s in self.shifts:
                    if (i, t, s) in start_vals:
                        value_cons = start_vals[i, t, s]
                    else:
                        value_cons = 0

                    if (i, t, s) in start_vals:
                        self.model.chgCoeff(self.cons_demand[t, s], self.lmbda[i, 1], value_cons)
        self.model.update()

    def addCol(self, itr, schedules_perf):
        for i in self.nurses:
            for t in self.days:
                for s in self.shifts:
                    if (i, t, s, itr + 1) in schedules_perf:
                        value_cons = schedules_perf[i, t, s, itr + 1]
                    else:
                        value_cons = 0

                    if (i, t, s, itr + 1) in schedules_perf:
                        self.model.chgCoeff(self.cons_demand[t, s], self.lmbda[i, itr + 1], value_cons)
        self.model.update()

    def finalSolve(self, timeLimit):
        try:
            self.model.setParam('TimeLimit', timeLimit)
            self.model.Params.MIPGap = 1e-4
            self.model.Params.OutputFlag = 1
            for var in self.model.getVars():
                if "lmbda" in var.VarName:
                    var.VType = gu.GRB.BINARY
            self.model.update()
            self.model.optimize()
            if self.model.status == gu.GRB.OPTIMAL:
                print("*" * (self.output_len + 2))
                print("*{:^{output_len}}*".format("***** Optimal solution found *****", output_len=self.output_len))
                print("*" * (self.output_len + 2))
            else:
                print("*" * (self.output_len + 2))
                print("*{:^{output_len}}*".format("***** No optimal solution found *****", output_len=self.output_len))
                print("*" * (self.output_len + 2))
        except gu.GurobiError as e:
            print('Error code ' + str(e.errno) + ': ' + str(e))

    def solveModel(self, timeLimit):
        try:
            self.model.setParam('TimeLimit', timeLimit)
            self.model.Params.OutputFlag = 1
            self.model.Params.IntegralityFocus = 1
            self.model.Params.FeasibilityTol = 1e-7
            self.model.Params.BarConvTol = 0.0
            self.model.setParam('ConcurrentMIP', 2)
            self.model.optimize()
        except gu.GurobiError as e:
            print('Error code ' + str(e.errno) + ': ' + str(e))

    def solveRelaxModel(self):
        try:
            self.model.Params.OutputFlag = 1
            # self.model.Params.Method = 2
            # self.model.Params.Crossover = 0
            for v in self.model.getVars():
                v.setAttr('vtype', 'C')
                v.setAttr('lb', 0.0)
            print('Variables relaxed')
            self.model.optimize()
        except gu.GurobiError as e:
            print('Error code ' + str(e.errno) + ': ' + str(e))

    def branch_var(self):
        import re
        import math

        if self.model.status != gu.GRB.OPTIMAL:
            raise Exception("Master problem could not find an optimal solution.")

        lambda_vars = self.model.getVars()
        fractional_values = {}

        for var in lambda_vars:
            if 'lmbda' in var.varName:
                value = var.x
                if abs(value - round(value)) > 1e-6:
                    match = re.search(r'\[(\d+),(\d+)\]', var.varName)
                    if match:
                        i = int(match.group(1))
                        r = int(match.group(2))
                        fractional_values[(i, r)] = value

        if not fractional_values:
            print("No fractional lambdas.")
            return None, None

        most_fractional_var = max(
            fractional_values.items(),
            key=lambda x: abs(x[1] - round(x[1]))
        )
        i, r = most_fractional_var[0]
        var_value = most_fractional_var[1]

        print(f'Most fractional lambda is for i: {i}, r: {r} with value {var_value}')

        return i, r
