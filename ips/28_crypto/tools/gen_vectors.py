#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Verify NIST FIPS-197 / SP 800-38A / FIPS 180-4 / RFC 4231 test vectors and
emit deterministic padding-edge SHA/HMAC reference digests using Python hashlib/hmac.
"""
import hashlib
import hmac as py_hmac


# ---- Minimal independent Python AES reference (byte-oriented, FIPS-197) ----
def _xtime(a: int) -> int:
    return ((a << 1) ^ 0x1B) & 0xFF if (a & 0x80) else (a << 1) & 0xFF


def _gmul(a: int, b: int) -> int:
    p = 0
    for _ in range(8):
        if b & 1:
            p ^= a
        a = _xtime(a)
        b >>= 1
    return p


def _build_sbox():
    s = [0] * 256
    for x in range(256):
        inv = 0 if x == 0 else pow(x, 254, 0x11B) if False else 1
        if x != 0:
            inv = 1
            for _ in range(254):
                inv = _gmul(inv, x)
        r = inv
        for k in (1, 2, 3, 4):
            r ^= ((inv << k) | (inv >> (8 - k))) & 0xFF
        s[x] = r ^ 0x63
    return s


SBOX = _build_sbox()


def aes_expand_key(key: bytes):
    nk = len(key) // 4
    nr = nk + 6
    w = [list(key[4 * i:4 * i + 4]) for i in range(nk)]
    rcon = 1
    for i in range(nk, 4 * (nr + 1)):
        tmp = w[i - 1][:]
        if i % nk == 0:
            tmp = [SBOX[tmp[1]] ^ rcon, SBOX[tmp[2]], SBOX[tmp[3]], SBOX[tmp[0]]]
            rcon = _xtime(rcon)
        elif nk == 8 and (i % nk) == 4:
            tmp = [SBOX[b] for b in tmp]
        w.append([w[i - nk][j] ^ tmp[j] for j in range(4)])
    return [bytes(sum(w[4 * r:4 * r + 4], [])) for r in range(nr + 1)]


def aes_encrypt_block(key: bytes, pt: bytes) -> bytes:
    rks = aes_expand_key(key)
    nr = len(rks) - 1
    st = bytearray(a ^ b for a, b in zip(pt, rks[0]))
    for rnd in range(1, nr + 1):
        st = bytearray(SBOX[b] for b in st)
        # ShiftRows (col-major: byte r + 4*c)
        sr = bytearray(16)
        for c in range(4):
            for r in range(4):
                sr[4 * c + r] = st[4 * ((c + r) % 4) + r]
        st = sr
        if rnd != nr:
            mc = bytearray(16)
            for c in range(4):
                a0, a1, a2, a3 = st[4 * c:4 * c + 4]
                mc[4 * c + 0] = _gmul(2, a0) ^ _gmul(3, a1) ^ a2 ^ a3
                mc[4 * c + 1] = a0 ^ _gmul(2, a1) ^ _gmul(3, a2) ^ a3
                mc[4 * c + 2] = a0 ^ a1 ^ _gmul(2, a2) ^ _gmul(3, a3)
                mc[4 * c + 3] = _gmul(3, a0) ^ a1 ^ a2 ^ _gmul(2, a3)
            st = mc
        st = bytearray(a ^ b for a, b in zip(st, rks[rnd]))
    return bytes(st)


def main():
    # 1. Verify FIPS-197 Appendix B / C
    pt = bytes.fromhex("00112233445566778899aabbccddeeff")
    assert aes_encrypt_block(bytes.fromhex("000102030405060708090a0b0c0d0e0f"), pt).hex() == "69c4e0d86a7b0430d8cdb78070b4c55a"
    assert aes_encrypt_block(bytes.fromhex("000102030405060708090a0b0c0d0e0f1011121314151617"), pt).hex() == "dda97ca4864cdfe06eaf70a0ec0d7191"
    assert aes_encrypt_block(bytes.fromhex("000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f"), pt).hex() == "8ea2b7ca516745bfeafc49904b496089"

    # 2. Verify NIST SP 800-38A CBC-AES128 and CTR-AES128
    key128 = bytes.fromhex("2b7e151628aed2a6abf7158809cf4f3c")
    blocks = [
        bytes.fromhex("6bc1bee22e409f96e93d7e117393172a"),
        bytes.fromhex("ae2d8a571e03ac9c9eb76fac45af8e51"),
        bytes.fromhex("30c81c46a35ce411e5fbc1191a0a52ef"),
        bytes.fromhex("f69f2445df4f9b17ad2b417be66c3710"),
    ]
    iv_cbc = bytes.fromhex("000102030405060708090a0b0c0d0e0f")
    exp_cbc = [
        "7649abac8119b246cee98e9b12e9197d",
        "5086cb9b507219ee95db113a917678b2",
        "73bed6b8e3c1743b7116e69e22229516",
        "3ff1caa1681fac09120eca307586e1a7",
    ]
    prev = iv_cbc
    for blk, exp in zip(blocks, exp_cbc):
        ct = aes_encrypt_block(key128, bytes(a ^ b for a, b in zip(blk, prev)))
        assert ct.hex() == exp
        prev = ct

    ctr = int.from_bytes(bytes.fromhex("f0f1f2f3f4f5f6f7f8f9fafbfcfdfeff"), "big")
    exp_ctr = [
        "874d6191b620e3261bef6864990db6ce",
        "9806f66b7970fdff8617187bb9fffdff",
        "5ae4df3edbd5d35e5b4f09020db03eab",
        "1e031dda2fbe03d1792170a0f3009cee",
    ]
    for blk, exp in zip(blocks, exp_ctr):
        ks = aes_encrypt_block(key128, ctr.to_bytes(16, "big"))
        ct = bytes(a ^ b for a, b in zip(blk, ks))
        assert ct.hex() == exp
        ctr += 1

    # 3. Print padding-edge SHA vectors (0, 55, 56, 63, 64, 111, 112, 119, 120, 127, 128)
    for n in [0, 3, 55, 56, 63, 64, 111, 112, 119, 120, 127, 128]:
        msg = bytes([(i * 13 + 7) & 0xFF for i in range(n)])
        print(f"SHA256[{n}] = {hashlib.sha256(msg).hexdigest()}")
        print(f"SHA224[{n}] = {hashlib.sha224(msg).hexdigest()}")
        print(f"SHA512[{n}] = {hashlib.sha512(msg).hexdigest()}")
        print(f"SHA384[{n}] = {hashlib.sha384(msg).hexdigest()}")

    # 4. RFC 4231 Test Case 1 & 2 (HMAC-SHA256 / 224 / 384 / 512)
    k1 = bytes.fromhex("0b" * 20)
    d1 = b"Hi There"
    print("RFC4231_TC1_256 =", py_hmac.new(k1, d1, hashlib.sha256).hexdigest())
    print("RFC4231_TC1_224 =", py_hmac.new(k1, d1, hashlib.sha224).hexdigest())
    print("RFC4231_TC1_384 =", py_hmac.new(k1, d1, hashlib.sha384).hexdigest())
    print("RFC4231_TC1_512 =", py_hmac.new(k1, d1, hashlib.sha512).hexdigest())

    k2 = b"Jefe"
    d2 = b"what do ya want for nothing?"
    print("RFC4231_TC2_256 =", py_hmac.new(k2, d2, hashlib.sha256).hexdigest())

    # Edge-length HMAC key (64B and 0B)
    print("HMAC256_K64 =", py_hmac.new(bytes(range(64)), b"abc", hashlib.sha256).hexdigest())
    print("HMAC256_K0  =", py_hmac.new(b"", b"abc", hashlib.sha256).hexdigest())
    print("ALL PYTHON KAT CHECKS PASSED")


if __name__ == "__main__":
    main()
