ARG TAG=latest
FROM salsicha/cyberphysics:pantiltros-${TAG}

COPY scripts/ /opt/turret_sim/scripts/
COPY scenarios/ /opt/turret_sim/scenarios/
COPY validation/ /opt/turret_sim/validation/

RUN chmod +x /opt/turret_sim/scripts/*.py
