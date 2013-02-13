%%
%% Copyright (C) 2011 by Moritz Tenorth
%%
%% This module provides methods for creating object instances in KnowRob
%% based on perceptual information.
%%
%% This program is free software; you can redistribute it and/or modify
%% it under the terms of the GNU General Public License as published by
%% the Free Software Foundation; either version 3 of the License, or
%% (at your option) any later version.
%%
%% This program is distributed in the hope that it will be useful,
%% but WITHOUT ANY WARRANTY; without even the implied warranty of
%% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%% GNU General Public License for more details.
%%
%% You should have received a copy of the GNU General Public License
%% along with this program.  If not, see <http://www.gnu.org/licenses/>.
%%


:- module(prac_client,
    [
      infer_roles/2,
      infer_roles_for_actionclass/2,
      action_role/3
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('thea/owl_parser')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('knowrob_owl')).

:- rdf_db:rdf_register_ns(rdf,     'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(rdfs,    'http://www.w3.org/2000/01/rdf-schema#',       [keep(true)]).
:- rdf_db:rdf_register_ns(owl,     'http://www.w3.org/2002/07/owl#',              [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://ias.cs.tum.edu/kb/knowrob.owl#',       [keep(true)]).
:- rdf_db:rdf_register_ns(xsd,     'http://www.w3.org/2001/XMLSchema#',           [keep(true)]).

:-  rdf_meta
    infer_roles(r,-),
    infer_roles_for_actionclass(r,-),
    action_role(+,r,r).


infer_roles_for_actionclass(ActionClass, Roles) :-
  rdf_has(ActionClass, rdfs:label, literal(type(xsd:string, Instruction))),
  infer_roles(Instruction, Roles).


infer_roles(Instruction, Roles) :-

  jpl_new('edu.tum.cs.ias.knowrob.prac.PracRosClient', [], Client),
  jpl_call(Client, callPracInferService, ['Flipping', Instruction], Res),
  jpl_call(Res, toArray, [], ResArray),
  jpl_array_to_list(ResArray, RolesList),
  findall([Role, Concept], (member(R, RolesList), action_role(R, Role, Concept)), Roles).


action_role(ActionRole, Role, Concept) :-
    jpl_get(ActionRole, role, Role),
    jpl_get(ActionRole, concept, Concept).



% roles:
%   -
%     role: Theme
%     concept: pancake.n.01
%   -
%     role: ActionVerb
%     concept: flip.v.08
%   -
%     role: Instrument
%     concept: spatula.n.01


