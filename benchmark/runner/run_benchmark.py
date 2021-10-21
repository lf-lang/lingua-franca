#!/usr/bin/env python3

import csv
import hydra
import logging
import multiprocessing
import omegaconf
import subprocess


log = logging.getLogger("run_benchmark")


@hydra.main(config_path="conf", config_name="default")
def main(cfg):
    # some aliases for convenience
    benchmark = cfg["benchmark"]
    target = cfg["target"]
    benchmark_name = benchmark["name"]
    target_name = target["name"]
    continue_on_error = cfg["continue_on_error"]
    test_mode = cfg["test_mode"]

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
    omegaconf.OmegaConf.register_new_resolver("args", resolve_args, replace=True)

    log.info(f"Running {benchmark_name} using the {target_name} target.")



    # perform some sanity checks
    if "validation_alias" in target:
        target_validation_name = target["validation_alias"]
    else:
        target_validation_name = target_name

    if target_validation_name not in benchmark["targets"]:
        log.warning(f"target {target_name} is not supported by the benchmark {benchmark_name}")
        return

    if not check_benchmark_target_config(benchmark, target_validation_name):
        if continue_on_error:
            return
        else:
            raise RuntimeError("Aborting because an error occurred")

    # prepare the benchmark
    for step in ["prepare", "copy", "gen", "compile"]:
        if target[step] is not None:
            _, code = execute_command(target[step])
            check_return_code(code, continue_on_error)

    # run the benchmark
    if target["run"] is not None:
        if test_mode:
            # run the command with a timeout of 1 second. We only want to test
            # if the command executes correctly, not if the full benchmark runs
            # correctly as this would take too long
            cmd = omegaconf.OmegaConf.to_object(target["run"])
            _, code = execute_command(["timeout", "1"] + cmd)
            # timeout returns 124 if the command executed correctly but the
            # timeout was exceeded
            if code != 0 and code != 124:
                raise RuntimeError(
                    f"Command returned with non-zero exit code ({code})"
                )
        else:
            output, code = execute_command(target["run"])
            check_return_code(code, continue_on_error)
            times = hydra.utils.call(target["parser"], output)
            write_results(times, cfg)
    else:
        raise ValueError(f"No run command provided for target {target_name}")


def check_return_code(code, continue_on_error):
    if code != 0:
        if continue_on_error:
            log.error(
                f"Command returned with non-zero exit code ({code})"
            )
        else:
            raise RuntimeError(
                f"Command returned with non-zero exit code ({code})"
            )

def check_benchmark_target_config(benchmark, target_name):
    benchmark_name = benchmark["name"]
    # keep a list of all benchmark parameters
    bench_params = list(benchmark["params"].keys())

    # collect all parameters used in target command arguments
    used_params = set()
    target_cfg = benchmark["targets"][target_name]
    for arg_type in ["gen_args", "compile_args", "run_args"]:
        if arg_type in target_cfg and target_cfg[arg_type] is not None:
            for param in target_cfg[arg_type].keys():
                if param not in bench_params:
                    log.error(f"{param} is not a parameter of the benchmark!")
                    return False
                used_params.add(param)

    for param in bench_params:
        if param not in used_params:
            log.warning(f"The benchmark parameter {param} is not used in any command")

    return True


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
        cmd_log = logging.getLogger(command[0])
        while True:
            nextline = process.stdout.readline()
            if nextline == "" and process.poll() is not None:
                break
            else:
                output.append(nextline)
                cmd_log.info(nextline.rstrip())

        code = process.returncode

    return output, code


def write_results(times, cfg):
    row = {
        "benchmark": cfg["benchmark"]["name"],
        "target": cfg["target"]["name"],
        "total_iterations": cfg["iterations"],
        "threads": cfg["threads"],
        "iteration": None,
        "time_ms": None,
    }
    # also add all parameters and their values
    row.update(cfg["benchmark"]["params"])
    if "params" in cfg["target"]:
        row.update(cfg["target"]["params"])

    with open("results.csv", "w", newline="") as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=row.keys())
        writer.writeheader()
        i = 0
        for t in times:
            row["iteration"] = i
            row["time_ms"] = t
            writer.writerow(row)
            i += 1


if __name__ == "__main__":
    main()
