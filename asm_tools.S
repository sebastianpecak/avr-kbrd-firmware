#define inTokenCount r23

; ---------------------------------
; local data
; ---------------------------------
    .data
    .extern inPacketCount

; ----------------------------------------------------------------------
; init_in_token_count
; ----------------------------------------------------------------------
    .text
    .global init_in_token_count
    .type init_in_token_count, @function

init_in_token_count:
    ldi inTokenCount, 0
    sts inPacketCount, inTokenCount
    ret

; ----------------------------------------------------------------------
; load_in_token_count
; ----------------------------------------------------------------------
    .text
    .global load_in_token_count
    .type load_in_token_count, @function

load_in_token_count:
    sts inPacketCount, inTokenCount
    ret