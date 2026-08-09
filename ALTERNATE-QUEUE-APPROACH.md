> **Status:** absorbed into `QUEUE-PROTOCOL.md` v4 (2026-08-09) as the base
> wire language, with the response widened to two sequences and the recovery
> rules added. Implement from QUEUE-PROTOCOL.md, not from this sketch.

New Queue Language.

There is a single command that gets sent for all queue activity.  Responses also come back in a queue,
and a value is not deleted from the response queue until the master acknowledges it has the response by ID.

A command consists of:
1. An optional command
2. an optional sequence ID to delete from the response queue.

A response consists of
1. a sequence number of the enqueued command, one was appended.
2. the status of the head of the response queue, or empty if there are no more responses to de-queue.

a command sequence number is a value between 1 and 31 - this can fit in 5 bits, and leaves 0 as a reserved value indicating no deletion. 

# Command Frame

4 Bytes:
1. a bitfield:
    1. 3 bits of flags (send twice, expect response, set DTR before command)
    1. 5 bits of sequence - if Non-0 this indicates that the client wishes to delete the head of the response queue, but only if its sequence matches the value.
1. DALI address
1. DALI command 
1. The value of DTR0.  This byte is optional, and only needs to be sent if the DTR bit is set

Modes in which this can operate:
1. Sending a command, but no deletion - send 3 or 4 bytes of command, but set SEQ to 0
1. Sending a command, _and_ deleting something - Same as above, but set a sequence
1. only deleting something - only send the first byte - subsequent bytes are ommitted.  The status bits are all 0.


# Response queue
2 Bytes:
1. a bitfield:
    1. 5 bits of sequence - if non-zero this indicates that there is a completed response in the queue.  Zero means that the queue is emtpy.
    1. 3 bits of status for the respnose. 
        1. success/fail - Indicates whether the command succeeded/failed
        1. enqueue failure - if set, the command queue was full, and the command was not enqueued.
1. the response byte - or error code if fail was incorrect.


# Method of operation
Whenever the master wishes to interact with the slave, it sends a custom FC (0x40) - properly encoded with its wishes.  The slave enqueues the command for exectuion.  It only allows one executing and one pending command at any time.  A request to enqueue a new command while one is already pending results a bit being set in the response.

This is intended to be as efficient as possible.  If it has a pending command to run, it keeps on attempting to send it until it succeeds.  Each time a request is made, the status of the completion queue head is sent back to the master.  The master claims results from the queue and deletes them when it has the response. 
