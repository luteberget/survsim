import csv


with open("benchmark.csv") as f:
    reader = csv.DictReader(f)
    for x in reader:
        bound = max(float(x['gurobi_30s--bound']), float(x['highs_30s--bound']))
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
            if xs[i][0] == '-':
                xs[i] = xs[i][1:]
                if "." in xs[i]:
                    xs[i] = xs[i][:(xs[i].index("."))]
            xs[i] = "$" + xs[i] + "$"
        print(str.join(' & ',xs).replace("inf","\\infty") + " \\\\ ")

