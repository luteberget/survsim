import csv
from collections import defaultdict


with open("benchmark.csv") as f:
    reader = csv.DictReader(f)
    for x in reader:
        bound = max(float(x['gurobi_30s--bound']), float(x['highs_30s--bound']))
        best = min((float(x[y]) for y in ["highs_30s--obj","colgen_30s--obj","greedy--obj"]))
        xs = [ f"{x['filename']}\\_{x['instance_idx']}",
            x['contacts'],
            x['vehicles'],
            x['highs_30s--time'],
            x['highs_30s--obj'],
            x['colgen_30s--time'],
            x['colgen_30s--obj'],
            x['greedy--time'],
            x['greedy--obj'],
            str(bound)
        ]
        for i in range(1,len(xs)):
            bold = False
            if xs[i][0] == '-':
                bold = float(xs[i]) == best and i != len(xs)-1
                xs[i] = xs[i][1:]
                if "." in xs[i]:
                    xs[i] = xs[i][:(xs[i].index("."))]
            if bold:
                xs[i] = "\mathbf{" + xs[i] + "}"
            xs[i] = "$" + xs[i] + "$"
        print(str.join(' & ',xs).replace("inf","\\infty") + " \\\\ ")


solvers = ["highs_30s--obj", "colgen_30s--obj","greedy--obj","highs_30s_gi--obj","colgen_30s_gi--obj"]
#solvers = ["highs_30s--obj", "gurobi_30s--obj", "colgen_30s--obj","greedy--obj","highs_30s_gi--obj","gurobi_30s_gi--obj","colgen_30s_gi--obj"]
with open("benchmark.csv") as f:
    reader = csv.DictReader(f)
    cat = defaultdict(lambda: defaultdict(list))
    for x in reader:
        bound = max(float(x['gurobi_30s--bound']), float(x['highs_30s--bound']))
        #print({y: float(x[y]) for y in solvers})
        best = min((float(x[y]) for y in solvers))

        for solver in solvers:
            cat[x["filename"]][solver].append(-min(0, float(x[solver]))/-best)

print(["cat"] + solvers)
for category,cs in cat.items():
    xs = [category] + [ f"{sum(ss) / len(ss):.2f}" for solver,ss in cs.items()]
    print(str.join(' & ', xs) + " \\\\ ")



