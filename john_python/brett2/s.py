import openravepy as o
env = o.Environment()
env.Load("robots/pr2-beta-static.zae")
rob = env.GetRobots()[0]
