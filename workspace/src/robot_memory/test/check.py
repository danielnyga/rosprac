from pracmln.mln import MLN
from pracmln.mln.methods import LearningMethods, InferenceMethods
from pracmln.mln.database import Database, parse_db
from sys import stdout

LOGIC = "FirstOrderLogic"
GRAMMAR = "PRACGrammar"
INFERENCE_METHOD = InferenceMethods.EnumerationAsk
LEARNING_METHOD = LearningMethods.BPLL


def learn(mln_name, db_name):
    """
    Learns the weights for the given MLN path using the given database path
    :rtype : MLN
    :type mln_name : str
    :type db_name : str
    """
    mln_to_learn = MLN(LOGIC, GRAMMAR, mln_name)
    dbs = Database.load(mln_to_learn, db_name)
    learned_mln = mln_to_learn.learn(dbs, LEARNING_METHOD, multicore=True)
    print("")
    return learned_mln


def open_mln(mln_name):
    return MLN(LOGIC, GRAMMAR, mln_name)


def query(mln, closed_world_predicates, query_string, evidence, debug):
    """
    Queries the given mln and prints the result to the command line
    :type mln: MLN
    :type closed_world_assumption: list
    :type query_string: str
    :type evidence: str
    :rtype : float
    """
    db = parse_db(mln, evidence)[0]
    materialized_mln = mln.materialize(db)
    mrf = materialized_mln.ground(db)
    if debug:
        headline = "Query: " + query_string
        underline = reduce(lambda elem, lst: elem+lst, ["=" for i in range(len(headline))])
        print(underline)
        print(headline)
        print(underline+"\n")
    inference = INFERENCE_METHOD(mrf, query_string, cw_preds=closed_world_predicates, multicore=True)
    if debug:
        inference.mrf.print_evidence_vars()
    inference.run()
    results = inference.results
    if debug:
        print("")
    query_formula = mln.logic.parse_formula(query_string)
    for formula in results:
        remove_whitespace = lambda x : "".join(x.split())
        if remove_whitespace(str(formula)) == remove_whitespace(str(query_formula)):
            return results[formula]
    raise(Exception("Formula not found!"))


def print_to_cmd_line(mln):
    """
    prints the given MLN to the command line
    :type mln : MLN
    """
    mln.write(stdout, True)
    print("")


class TestSuite:
    def __init__(self):
        self.tests = []

    def add_test(self, query_string, evidence_string, minimum_probability, maximum_probability, closed_world_predicates, mln):
        self.tests.append({"mln" : mln, "query" : query_string, "evidence" : evidence_string,
                           "min" : minimum_probability, "max" : maximum_probability, 
                           "closed_world_predicates" : closed_world_predicates})

    def execute(self):
        self.results = []
        for test in self.tests:
            result = query(test["mln"], test["closed_world_predicates"] , test["query"], test["evidence"], True)
            test_ok = result>=test["min"] and result<=test["max"]
	    singleLineEvidence = test["evidence"].replace("\n", " \\n ")
            self.results.append([test_ok, result, test["min"], test["max"], test["query"], singleLineEvidence])

    def print_results(self):
        everything_ok = True
	maxQueryLength = reduce(max, [len(result[4]) for result in self.results])
        print("Success Result Min    Max    Query")
        for result in self.results:
            everything_ok = everything_ok and result[0]
            print(("{0:<7} {1:1.4f} {2:1.4f} {3:1.4f} {4:<" + str(maxQueryLength) + "} {5}").\
	    	format(str(result[0]), result[1], result[2], result[3], result[4], result[5]))
        print("\nEVERY TEST SUCCESSFUL? " + str(everything_ok))
