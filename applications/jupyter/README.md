# How to run jupyter-lab from a docker container with ros installed

Build the image cyberphysics/jupyter
```commandline
cd apps
make build_jupyter
```

Run from project root
```commandline
docker compose -f ./compositions/jupyter.yml up
```

In your browser navigate to
`http://127.0.0.1:8888/lab?token=docker_jupyter`

If you are redirected to the jupyter login page, insert the password
`docker_jupyter`
