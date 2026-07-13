# Scheduler Feature

The Scheduler feature runs user-defined actions at a selected date and time, sunrise, or sunset, or when an event arrives on the existing `event` message pipe.

Rules can start at a selected time, sunrise, or sunset, using the station position from Preferences. They can be one-shot or recurring daily or monthly. Daily recurrence can be limited to selected weekdays. Monthly recurrence clamps invalid dates to the last day of shorter months, and missed startup occurrences are skipped.

Time rules can also have an optional duration in seconds, minutes, or hours, or run until the next sunset. When set, actions that start acquisition, file sink recording, compatible channel recording/scanning, or a feature are automatically stopped after the duration elapses or sunset is reached.

Event rules match `MainCore::MsgEvent` messages by event type, optional source channel/feature, optional data regular expression, and optional delay in seconds or minutes. Delayed actions re-check that the rule still exists and is enabled before they run.

Actions can load device-set presets, override center frequency, start or stop acquisition, start or stop file sinks, start or stop features, run a detached command, or speak text when text-to-speech support is available. Command and speech fields support `${rule}`, `${trigger}`, `${dateTime}`, `${event}`, `${source}`, `${data}`, and `${data.name}` substitutions. Event data field substitutions parse comma-separated `name=value` pairs from the event data string.
