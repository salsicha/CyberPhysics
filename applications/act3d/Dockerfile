# Reuse the repository's pinned CUDA PyTorch runtime. Act3D imports its own
# upstream model; the PerAct model is not used by this application.
ARG TAG=latest
FROM salsicha/cyberphysics:peract-${TAG}
ARG ACT3D_REF=5004650a9771e7618f24f36a876bcb38e0fc068a
USER root
RUN git clone --filter=blob:none --no-checkout https://github.com/zhouxian/act3d-chained-diffuser.git /opt/act3d \
 && cd /opt/act3d && git sparse-checkout set model \
 && git checkout ${ACT3D_REF} && rm -rf .git
RUN pip install --no-cache-dir \
    openai-clip==1.0.1 transformers==4.57.6 safetensors==0.7.0
ENV PYTHONPATH=/workspace/act3d_scripts:/workspace/peract_scripts \
    ACT3D_SOURCE=/opt/act3d HF_HOME=/workspace/cache/huggingface \
    XDG_CACHE_HOME=/workspace/cache PYTHONDONTWRITEBYTECODE=1
COPY scripts /workspace/act3d_scripts
USER 1234:1234
CMD ["python", "/workspace/act3d_scripts/policy_server.py"]
