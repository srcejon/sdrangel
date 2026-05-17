# Scheduler Feature

The Scheduler feature runs user-defined actions either at a selected date and time or when an event arrives on the existing `event` message pipe.

Rules can be one-shot or recurring daily, weekly, or monthly. Monthly recurrence clamps invalid dates to the last day of shorter months, and missed startup occurrences are skipped.

Event rules match `MainCore::MsgEvent` messages by event type, optional source channel/feature, optional data regular expression, and optional delay in seconds or minutes. Delayed actions re-check that the rule still exists and is enabled before they run.

Actions can load device-set presets, override center frequency, start or stop acquisition, start or stop file sinks, start or stop features, run a detached command, or speak text when text-to-speech support is available. Command and speech fields support `${rule}`, `${trigger}`, `${dateTime}`, `${event}`, `${source}`, and `${data}` substitutions.
