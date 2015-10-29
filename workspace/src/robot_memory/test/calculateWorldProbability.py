#!/usr/bin/python2

from pracmln.mln import MLN
from pracmln.mln.database import Database, parse_db
from numpy import exp, log


def main():
    mln = MLN(mlnfile="learnt_mlns/learnt_StateMachine_before_learning.mln")
    mln.write()
    dbs = Database.load(mln, "learnt_mlns/train.db", True)
    materialized = mln.materialize(*dbs)
    materialized.write()

    db_number = 0
    ll_sum = 0
    for db in dbs:
        log_likelihood = get_log_likelihood_for_database(db, materialized)
        print("log likelihood for db " + str(db_number) + ": " + str(log_likelihood))
        ll_sum += log_likelihood
        db_number += 1
    print("Overall log likelihood: " + str(ll_sum))


def get_log_likelihood_for_database(database, materialized_mln):
    mrf = materialized_mln.ground(database)
    true_db_atoms = set([str(atom_truth_pair[0]) for atom_truth_pair in database if atom_truth_pair[1] == 1.0])
    world_in_database = [0]*len(list(mrf.gndatoms))
    for atom in mrf.gndatoms:
        if str(atom) in true_db_atoms:
            world_in_database[atom.idx] = 1
    normalization_constant = 0
    nominator_for_database_world = 0
    for world_pair in mrf.iterallworlds():
        world = world_pair[1]
        weighted_sum = 0
        for gnd_formula in mrf.itergroundings():
            weighted_sum += gnd_formula(world)*gnd_formula.weight
        if world == world_in_database:
            nominator_for_database_world = weighted_sum
        normalization_constant += exp(weighted_sum)
    return nominator_for_database_world-log(normalization_constant)


if __name__ == "__main__":
    main()
