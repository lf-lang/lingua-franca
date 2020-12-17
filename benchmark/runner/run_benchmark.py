#!/usr/bin/env python3

import hydra
import logging
import multiprocessing
import omegaconf
import subprocess
import sys


log = logging.getLogger(__name__)


@hydra.main(config_path="conf", config_name="default")
def main(cfg):
    # some aliases for convenience
    benchmark = cfg["benchmark"]
    target = cfg["target"]
    benchmark_name = benchmark["name"]
    target_name = target["name"]

    # initialize the thread number if not specified
    if cfg["threads"] is None:
        cfg["threads"] = multiprocessing.cpu_count()

    # a function for resolving 'args:' in the configuration. We need the
    # closure here in order to have access to 'cfg'
    def resolve_args(config_key):
        # lookup the specified arguments in the configuration tree
        cfg_args = cfg
        for key in config_key.split("."):
            cfg_args = cfg_args[key]

        res = []
        if cfg_args is None:
            return res

        params = benchmark["params"]
        for k, v in cfg_args.items():
            value = str(params[k])
            for i in v:
                res.append(i.replace("<value>", value))

        return res

    # register the resolver for 'args:'

    # HACK: When using multirun, we need to update the resolver for the new
    # 'cfg.  However, OmegaConf throws an exception when the resolver was
    # already registerd in another run.  Since OmegaConf does not seem to offer
    # a way for deregistering a resolver we use this hack.
    try:
        omegaconf.basecontainer.BaseContainer._resolvers.pop("args")
    except KeyError:
        pass

    omegaconf.OmegaConf.register_resolver("args", resolve_args)

    log.info(f"Running {benchmark_name} using the {target_name} target.")

    # perform some sanity checks
    check_benchmark_target_config(benchmark, target_name)

    # prepare the benchmark
    for step in ["copy", "gen", "compile"]:
        if target[step] is not None:
            execute_command(target[step])

    # run the benchmark
    if target["run"] is not None:
        output = execute_command(target["run"])
        times = hydra.utils.call(target["parser"], output)
        print(times)
    else:
        raise ValueError(f"No run command provided for target {target_name}")


def check_benchmark_target_config(benchmark, target_name):
    benchmark_name = benchmark["name"]
    if target_name not in benchmark["targets"]:
        raise RuntimeError(
            f"target {target_name} is not supported by the benchmark {benchmark_name}"
        )
    # keep a list of all benchmark parameters
    bench_params = list(benchmark["params"].keys())

    # collect all parameters used in target command arguments
    used_params = set()
    target_cfg = benchmark["targets"][target_name]
    for arg_type in ["gen_args", "compile_args", "run_args"]:
        if arg_type in target_cfg and target_cfg[arg_type] is not None:
            for param in target_cfg[arg_type].keys():
                if param not in bench_params:
                    raise RuntimeError(f"{param} is not a parameter of the benchmark!")
                used_params.add(param)

    for param in bench_params:
        if param not in used_params:
            log.warning(f"The benchmark parameter {param} is not used in any command")


def execute_command(command):
    # the command can be a list of lists due to the way we use an omegaconf
    # resolver to determine the arguments. We need to flatten the command list
    # first. We also need to touch each element individually to make sure that
    # the resolvers are called.
    cmd = []
    for i in command:
        if isinstance(i, list) or isinstance(i, omegaconf.listconfig.ListConfig):
            cmd.extend(i)
        else:
            cmd.append(str(i))

    cmd_str = " ".join(cmd)
    log.info(f"run command: {cmd_str}")

    # run the command while printing and collecting its output
    output = []
    with subprocess.Popen(
        cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True
    ) as process:
        while True:
            nextline = process.stdout.readline()
            if nextline == "" and process.poll() is not None:
                break
            else:
                output.append(nextline)
                sys.stdout.write(nextline)

        if process.returncode != 0:
            raise RuntimeError(
                f"Command returned with non-zero exit code ({process.returncode})"
            )

    return output


if __name__ == "__main__":
    main()
