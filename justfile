set shell := ["bash", "-eu", "-o", "pipefail", "-c"]

default:
    @just --list

bootstrap:
    ./scripts/bootstrap.sh

doctor:
    ./scripts/doctor.sh

test-e2e:
    ./scripts/test-e2e.sh

setup-template:
    just -d arti-template setup

setup-data:
    just -d articraft_data setup

test-template:
    just -d arti-template smoke-tests

test-data:
    just -d articraft_data smoke-tests

eval command *args:
    ./scripts/eval.sh {{quote(command)}} {{args}}

viewer:
    ./scripts/viewer.sh

t2-authoring-pilot *args:
    python exp/scripts/run_t2_authoring_pilot.py {{args}}

t4-editability-protocol *args:
    arti-template/.venv/bin/python exp/scripts/prepare_t4_distributional_protocol.py {{args}}
