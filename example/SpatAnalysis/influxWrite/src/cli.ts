import commandLineArgs from 'command-line-args';
import commandLineUsage from 'command-line-usage';
import { UnitBasedTimeValue, TimeUnit, TimeValue} from './time';
import { LogLevel } from './util';

//---------------------------------------------------------------------//
// Command Line Arguments Helper Functions                             //
//---------------------------------------------------------------------//

/**
 * Function to convert a string into a LogLevel for command line argument parsing.
 * Returns null if the input is malformed.
 * @param logging the raw command line argument
 */
export function loggingCLAType(logging: string): LogLevel | null {
    if (logging in LogLevel) {
        type LevelString = keyof typeof LogLevel;
        return LogLevel[logging as LevelString];
    }  else {
        return null;
    }
}

/**
 * Function to convert a string into a UnitBasedTimeValue for command line argument parsing
 * Returns null if the input is malformed.
 * @param logging the raw command line argument
 */
export function unitBasedTimeValueCLAType(timeout: string): TimeValue | null {
    let duration:number;
    let units:TimeUnit;
    let wholeTimeoutPattern = /^[0-9]+\s+[a-z]+$/;
    if (wholeTimeoutPattern.test(timeout)) {
        let durationPattern = /^[0-9]+/;
        let unitsPattern = /[a-z]+$/;
        
        let stringDuration = durationPattern.exec(timeout);
        if (stringDuration !== null) {
            duration = parseInt(stringDuration[0]);
        } else {
            // Duration is not well formed.
            return null;
        }

        // Test if the units are a valid TimeUnits
        let stringUnits = unitsPattern.exec(timeout);
        if (stringUnits !== null && (stringUnits[0] in TimeUnit)){
            type TimeUnitString = keyof typeof TimeUnit;
            units = TimeUnit[stringUnits[0] as TimeUnitString]
        } else {
            // Units are not well formed.
            return null;
        }
        return new UnitBasedTimeValue(duration, units);
    } else {
        // Duration and units are not well formed.
        return null;
    }
}

/**
 * Function to convert a string into a boolean for command line argument parsing.
 * Returns null if the input is malformed.
 * Note that the command-line-arguments module's built in boolean type is
 * actually a flag that is either absent or true. https://github.com/75lb/command-line-args/wiki/Notation-rules
 * We need this custom boolean parsing because our command line arguments
 * are true, false, or absent.
 * @param logging the raw command line argument
 */
export function booleanCLAType(bool: string): boolean | null {
    if (bool === "true" ) {
        return true;
    } else if (bool === "false") {
        return false;
    } else {
        return null;
    }
}

//---------------------------------------------------------------------//
// Exported CLI support                                                //
//---------------------------------------------------------------------//

/**
 * The type returned by the commandLineArguments function. This type must change
 * if the CommandLineOptionDefs changes.
 */
export type ProcessedCommandLineArgs = {fast: boolean| undefined,
    keepalive: boolean | undefined, timeout: UnitBasedTimeValue | null | undefined,
    logging: LogLevel | undefined, help: boolean}


export type CommandLineOptionSpec = Array<{name: string, alias?: string, 
    type: (arg0: string) => unknown, typeLabel?: string, description: string}>;

/**
 * Configuration for command line arguments.
 * If this configuration changes, the ProcessedCommandLineArgs type must
 * change too.
 */
export const CommandLineOptionDefs: CommandLineOptionSpec = [
    { name: 'keepalive', alias: 'k', type: booleanCLAType, typeLabel: '{underline [true | false]}',
    description: 'Specifies whether to stop execution if there are no events to process. ' +
        'This defaults to false, meaning that the program will stop executing when ' +
        'there are no more events on the event queue. If you set this to true, then ' +
        'the program will keep executing until either the timeout logical time is ' +
        'reached or the program is externally killed. If you have physical actions, ' +
        'it usually makes sense to set this to true.'
    },
    { name: 'fast', alias: 'f', type: booleanCLAType, typeLabel: '{underline [true | false]}',
        description: 'Specifies whether to wait for physical time to match logical time. ' +
            'The default is false. If this is true, then the program will execute as fast ' +
            'as possible, letting logical time advance faster than physical time.'
    },
    { name: 'logging', alias: 'l', type: loggingCLAType, typeLabel: '{underline [ERROR | WARN | INFO | LOG | DEBUG]}',
        description: 'The level of diagnostic messages about execution to print to the ' +
            'console. A message will print if this parameter is greater than or equal to ' +
            'the level of the message (ERROR < WARN < INFO < LOG < DEBUG).'
    },
    { name: 'timeout', alias: 'o', type: unitBasedTimeValueCLAType, typeLabel: "{underline '<duration> <units>'}",
        description: 'Stop execution when logical time has advanced by the specified <duration>. ' +
            'The units can be any of nsec, usec, msec, sec, minute, hour, day, week, or the plurals ' +
            'of those. For the duration and units of a timeout argument to be parsed correctly as a ' +
            'single value, these should be specified in quotes with no leading or trailing space ' +
            "'(eg '5 sec')."
    },
    { name: 'help', alias: 'h', type: Boolean, 
        description: 'Print this usage guide. The program will not execute if this flag is present.'
    }
];

/**
 * Configuration for command line argument usage information.
 * Note: The order of the elements in the list is important.
 */
export const CommandLineUsageDefs : Array<{header: string, content?: string, 
    optionList?: CommandLineOptionSpec}> = [
        {
            header: 'Command Line Usage for TypeScript Reactors',
            content: 'This generated program understands the following command-line arguments, ' +
                'each of which has a short form (one character) and a long form. ' + 
                'If provided, a command line argument will override whatever value ' +
                'the corresponding target property had specified in the source .lf file.'
        },
        {
            header: 'Options',
            optionList: CommandLineOptionDefs
        }
    ];
