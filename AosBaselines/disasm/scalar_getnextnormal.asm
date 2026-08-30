; Assembly listing for method AosBaselines.ScalarDepthRefiner`4[BepuPhysics.Collidables.Cylinder,AosBaselines.CylinderSupportScalar,BepuPhysics.Collidables.Cylinder,AosBaselines.CylinderSupportScalar]:GetNextNormal(byref,byref,byref,bool,byref,byref,float,float,byref) (FullOpts)
; Emitting BLENDED_CODE for generic X64 + VEX on Windows
; FullOpts code
; optimized code
; rsp based frame
; partially interruptible
; No PGO data
; 0 inlinees with PGO data; 58 single block inlinees; 55 inlinees without PGO data

G_M000_IG01:                ;; offset=0x0000
       push     r15
       push     r14
       push     r13
       push     r12
       push     rdi
       push     rsi
       push     rbp
       push     rbx
       sub      rsp, 424
       vmovaps  xmmword ptr [rsp+0x190], xmm6
       vmovaps  xmmword ptr [rsp+0x180], xmm7
       vmovaps  xmmword ptr [rsp+0x170], xmm8
       vmovaps  xmmword ptr [rsp+0x160], xmm9
       vmovaps  xmmword ptr [rsp+0x150], xmm10
       vmovaps  xmmword ptr [rsp+0x140], xmm11
       vmovaps  xmmword ptr [rsp+0x130], xmm12
       vmovaps  xmmword ptr [rsp+0x120], xmm13
       vmovaps  xmmword ptr [rsp+0x110], xmm14
       vmovaps  xmmword ptr [rsp+0x100], xmm15
       mov      rax, bword ptr [rsp+0x210]
       mov      r10, bword ptr [rsp+0x218]
       vmovss   xmm0, dword ptr [rsp+0x220]
       vmovss   xmm1, dword ptr [rsp+0x228]
 
G_M000_IG02:                ;; offset=0x008F
       vmovsd   xmm2, qword ptr [r10]
       vinsertps xmm2, xmm2, dword ptr [r10+0x08], 40
       vxorps   xmm3, xmm3, xmm3
       vmaxss   xmm3, xmm3, xmm0
       vbroadcastss xmm3, xmm3
       vmulps   xmm2, xmm3, xmm2
       vmovaps  xmmword ptr [rsp+0xE0], xmm2
       vxorps   xmm3, xmm3, xmm3
       mov      r11d, -1
       xor      ebx, ebx
       vucomiss xmm3, xmm0
       cmovbe   r11d, ebx
       vmovd    xmm3, r11d
       vmovss   dword ptr [rsp+0x220], xmm0
       vsubss   xmm4, xmm1, xmm0
       vandps   xmm4, xmm4, xmm3
       vandnps  xmm1, xmm3, xmm1
       vorps    xmm1, xmm1, xmm4
       vmulss   xmm1, xmm1, xmm1
       vmovss   dword ptr [rsp+0xDC], xmm1
       test     r9b, r9b
       jne      G_M000_IG07
 
G_M000_IG03:                ;; offset=0x00FD
       lea      rdx, bword ptr [rcx+0x0C]
       mov      r8, rdx
       movsx    r9, byte  ptr [rcx+0x1C]
       vmovsd   xmm3, qword ptr [rcx]
       vinsertps xmm3, xmm3, dword ptr [rcx+0x08], 40
       vmovsd   qword ptr [rcx], xmm3
       vextractps dword ptr [rcx+0x08], xmm3, 2
       vmovsd   xmm3, qword ptr [r8]
       vinsertps xmm3, xmm3, dword ptr [r8+0x08], 40
       vmovsd   qword ptr [rcx+0x0C], xmm3
       vextractps dword ptr [rcx+0x14], xmm3, 2
       mov      byte  ptr [rcx+0x1C], 1
       lea      r8, bword ptr [rcx+0x20]
       mov      r9, r8
       mov      r11, rdx
       cmp      byte  ptr [r9+0x1C], 0
       sete     bl
       movzx    rbx, bl
       vmovsd   xmm3, qword ptr [rcx]
       vinsertps xmm3, xmm3, dword ptr [rcx+0x08], 40
       vmovsd   xmm4, qword ptr [r9]
       vinsertps xmm4, xmm4, dword ptr [r9+0x08], 40
       mov      esi, -1
       xor      edi, edi
       test     ebx, ebx
       cmove    esi, edi
       vmovd    xmm5, esi
       vpbroadcastd xmm5, xmm5
       vandps   xmm3, xmm3, xmm5
       vandnps  xmm4, xmm5, xmm4
       vorps    xmm3, xmm4, xmm3
       vmovsd   qword ptr [r9], xmm3
       vextractps dword ptr [r9+0x08], xmm3, 2
       vmovsd   xmm3, qword ptr [r11]
       vinsertps xmm3, xmm3, dword ptr [r11+0x08], 40
       vmovsd   xmm4, qword ptr [r9+0x0C]
       vinsertps xmm4, xmm4, dword ptr [r9+0x14], 40
       mov      r11d, -1
 
G_M000_IG04:                ;; offset=0x01B3
       test     ebx, ebx
       cmove    r11d, edi
       vmovd    xmm5, r11d
       vpbroadcastd xmm5, xmm5
       vandps   xmm3, xmm3, xmm5
       vandnps  xmm4, xmm5, xmm4
       vorps    xmm3, xmm4, xmm3
       vmovsd   qword ptr [r9+0x0C], xmm3
       vextractps dword ptr [r9+0x14], xmm3, 2
       mov      byte  ptr [r9+0x1C], 1
       lea      r9, bword ptr [rcx+0x40]
       mov      r11, r9
       cmp      byte  ptr [r11+0x1C], 0
       sete     bl
       movzx    rbx, bl
       vmovsd   xmm3, qword ptr [rcx]
       vinsertps xmm3, xmm3, dword ptr [rcx+0x08], 40
       vmovsd   xmm4, qword ptr [r11]
       vinsertps xmm4, xmm4, dword ptr [r11+0x08], 40
       mov      esi, -1
 
G_M000_IG05:                ;; offset=0x020F
       test     ebx, ebx
       cmove    esi, edi
       vmovd    xmm5, esi
       vpbroadcastd xmm5, xmm5
       vandps   xmm3, xmm3, xmm5
       vandnps  xmm4, xmm5, xmm4
       vorps    xmm3, xmm4, xmm3
       vmovsd   qword ptr [r11], xmm3
       vextractps dword ptr [r11+0x08], xmm3, 2
       vmovsd   xmm3, qword ptr [rdx]
       vinsertps xmm3, xmm3, dword ptr [rdx+0x08], 40
       vmovsd   xmm4, qword ptr [r11+0x0C]
       vinsertps xmm4, xmm4, dword ptr [r11+0x14], 40
       mov      edx, -1
 
G_M000_IG06:                ;; offset=0x0252
       test     ebx, ebx
       cmove    edx, edi
       vmovd    xmm5, edx
       vpbroadcastd xmm5, xmm5
       vandps   xmm3, xmm3, xmm5
       vandnps  xmm4, xmm5, xmm4
       vorps    xmm3, xmm4, xmm3
       vmovsd   qword ptr [r11+0x0C], xmm3
       vextractps dword ptr [r11+0x14], xmm3, 2
       mov      byte  ptr [r11+0x1C], 1
       mov      rbx, r9
       mov      r9, r8
       jmp      G_M000_IG19
 
G_M000_IG07:                ;; offset=0x0289
       movzx    r9, byte  ptr [rcx+0x1C]
       movzx    r11, byte  ptr [rcx+0x3C]
       and      r11d, r9d
       movzx    rbx, byte  ptr [rcx+0x5C]
       and      r11d, ebx
       test     r9d, r9d
       sete     r9b
       movzx    r9, r9b
       vmovsd   xmm3, qword ptr [rdx]
       vinsertps xmm3, xmm3, dword ptr [rdx+0x08], 40
       vmovsd   xmm4, qword ptr [rcx]
       vinsertps xmm4, xmm4, dword ptr [rcx+0x08], 40
       mov      ebx, -1
       xor      esi, esi
       test     r9d, r9d
       cmove    ebx, esi
       vmovd    xmm5, ebx
       vpbroadcastd xmm5, xmm5
       vandps   xmm3, xmm3, xmm5
       vandnps  xmm4, xmm5, xmm4
       vorps    xmm3, xmm4, xmm3
       vmovsd   qword ptr [rcx], xmm3
       vextractps dword ptr [rcx+0x08], xmm3, 2
       vmovsd   xmm3, qword ptr [r8]
       vinsertps xmm3, xmm3, dword ptr [r8+0x08], 40
       vmovsd   xmm4, qword ptr [rcx+0x0C]
       vinsertps xmm4, xmm4, dword ptr [rcx+0x14], 40
       mov      ebx, -1
 
G_M000_IG08:                ;; offset=0x0308
       test     r9d, r9d
       cmove    ebx, esi
       vmovd    xmm5, ebx
       vpbroadcastd xmm5, xmm5
       vandps   xmm3, xmm3, xmm5
       vandnps  xmm4, xmm5, xmm4
       vorps    xmm3, xmm4, xmm3
       vmovsd   qword ptr [rcx+0x0C], xmm3
       vextractps dword ptr [rcx+0x14], xmm3, 2
       mov      byte  ptr [rcx+0x1C], 1
       lea      r9, bword ptr [rcx+0x20]
       mov      rbx, r9
       cmp      byte  ptr [rbx+0x1C], 0
       sete     sil
       movzx    rsi, sil
       vmovsd   xmm3, qword ptr [rdx]
       vinsertps xmm3, xmm3, dword ptr [rdx+0x08], 40
       vmovsd   xmm4, qword ptr [rbx]
       vinsertps xmm4, xmm4, dword ptr [rbx+0x08], 40
       mov      edi, -1
       xor      ebp, ebp
       test     esi, esi
       cmove    edi, ebp
       vmovd    xmm5, edi
       vpbroadcastd xmm5, xmm5
       vandps   xmm3, xmm3, xmm5
       vandnps  xmm4, xmm5, xmm4
       vorps    xmm3, xmm4, xmm3
       vmovsd   qword ptr [rbx], xmm3
       vextractps dword ptr [rbx+0x08], xmm3, 2
       vmovsd   xmm3, qword ptr [r8]
       vinsertps xmm3, xmm3, dword ptr [r8+0x08], 40
       vmovsd   xmm4, qword ptr [rbx+0x0C]
       vinsertps xmm4, xmm4, dword ptr [rbx+0x14], 40
       mov      edi, -1
 
G_M000_IG09:                ;; offset=0x03A5
       test     esi, esi
       cmove    edi, ebp
       vmovd    xmm5, edi
       vpbroadcastd xmm5, xmm5
       vandps   xmm3, xmm3, xmm5
       vandnps  xmm4, xmm5, xmm4
       vorps    xmm3, xmm4, xmm3
       vmovsd   qword ptr [rbx+0x0C], xmm3
       vextractps dword ptr [rbx+0x14], xmm3, 2
       mov      byte  ptr [rbx+0x1C], 1
       lea      rbx, bword ptr [rcx+0x40]
       mov      rsi, rbx
       cmp      byte  ptr [rsi+0x1C], 0
       sete     dil
       movzx    rdi, dil
       vmovsd   xmm3, qword ptr [rdx]
       vinsertps xmm3, xmm3, dword ptr [rdx+0x08], 40
       vmovsd   xmm4, qword ptr [rsi]
       vinsertps xmm4, xmm4, dword ptr [rsi+0x08], 40
       mov      ebp, -1
       xor      r14d, r14d
       test     edi, edi
       cmove    ebp, r14d
       vmovd    xmm5, ebp
       vpbroadcastd xmm5, xmm5
       vandps   xmm3, xmm3, xmm5
       vandnps  xmm4, xmm5, xmm4
       vorps    xmm3, xmm4, xmm3
       vmovsd   qword ptr [rsi], xmm3
       vextractps dword ptr [rsi+0x08], xmm3, 2
       vmovsd   xmm3, qword ptr [r8]
       vinsertps xmm3, xmm3, dword ptr [r8+0x08], 40
       vmovsd   xmm4, qword ptr [rsi+0x0C]
       vinsertps xmm4, xmm4, dword ptr [rsi+0x14], 40
       mov      ebp, -1
 
G_M000_IG10:                ;; offset=0x0443
       test     edi, edi
       cmove    ebp, r14d
       vmovd    xmm5, ebp
       vpbroadcastd xmm5, xmm5
       vandps   xmm3, xmm3, xmm5
       vandnps  xmm4, xmm5, xmm4
       vorps    xmm3, xmm4, xmm3
       vmovsd   qword ptr [rsi+0x0C], xmm3
       vextractps dword ptr [rsi+0x14], xmm3, 2
       mov      byte  ptr [rsi+0x1C], 1
       test     r11d, r11d
       jne      SHORT G_M000_IG12
 
G_M000_IG11:                ;; offset=0x0473
       jmp      G_M000_IG19
 
G_M000_IG12:                ;; offset=0x0478
       vmovsd   xmm3, qword ptr [rcx+0x20]
       vinsertps xmm3, xmm3, dword ptr [rcx+0x28], 40
       vmovsd   xmm4, qword ptr [rcx]
       vinsertps xmm4, xmm4, dword ptr [rcx+0x08], 40
       vsubps   xmm5, xmm3, xmm4
       vmovsd   xmm6, qword ptr [rcx+0x40]
       vinsertps xmm6, xmm6, dword ptr [rcx+0x48], 40
       vsubps   xmm7, xmm4, xmm6
       vmovsd   xmm8, qword ptr [rdx]
       vinsertps xmm8, xmm8, dword ptr [rdx+0x08], 40
       vsubps   xmm4, xmm8, xmm4
       vsubps   xmm3, xmm8, xmm3
       vsubps   xmm6, xmm8, xmm6
       vpermilps xmm9, xmm5, 9
       vpermilps xmm10, xmm7, 18
       vmulps   xmm9, xmm10, xmm9
       vpermilps xmm5, xmm5, 18
       vpermilps xmm7, xmm7, 9
       vmulps   xmm5, xmm7, xmm5
       vsubps   xmm5, xmm9, xmm5
       vmovaps  xmm7, xmm2
       vsubps   xmm7, xmm8, xmm7
       vpermilps xmm8, xmm5, 9
       vpermilps xmm9, xmm7, 18
       vmulps   xmm8, xmm9, xmm8
       vpermilps xmm5, xmm5, 18
       vpermilps xmm7, xmm7, 9
       vmulps   xmm5, xmm7, xmm5
       vsubps   xmm5, xmm8, xmm5
       vinsertps xmm5, xmm5, xmm5, 56
       vinsertps xmm4, xmm4, xmm4, 56
       vmulps   xmm4, xmm4, xmm5
       vpermilps xmm7, xmm4, -11
       vaddps   xmm7, xmm7, xmm4
       vpermilps xmm4, xmm4, -86
       vaddps   xmm4, xmm4, xmm7
       vinsertps xmm3, xmm3, xmm3, 56
       vmulps   xmm3, xmm3, xmm5
       vpermilps xmm7, xmm3, -11
       vaddps   xmm7, xmm7, xmm3
       vpermilps xmm3, xmm3, -86
       vaddps   xmm3, xmm3, xmm7
       vinsertps xmm6, xmm6, xmm6, 56
       vmulps   xmm5, xmm6, xmm5
       vpermilps xmm6, xmm5, -11
       vaddps   xmm6, xmm6, xmm5
       vpermilps xmm5, xmm5, -86
       vaddps   xmm5, xmm5, xmm6
       vxorps   xmm6, xmm6, xmm6
       vucomiss xmm4, xmm6
       setae    r11b
       movzx    r11, r11b
       vucomiss xmm6, xmm3
       seta     sil
       movzx    rsi, sil
       and      r11d, esi
       vucomiss xmm3, xmm6
       setae    sil
       movzx    rsi, sil
       vucomiss xmm6, xmm5
 
G_M000_IG13:                ;; offset=0x059B
       seta     dil
       movzx    rdi, dil
       and      esi, edi
       vucomiss xmm5, xmm6
       setae    dil
       movzx    rdi, dil
       vucomiss xmm6, xmm4
       seta     bpl
       movzx    rbp, bpl
       and      edi, ebp
       mov      ebp, r11d
       or       ebp, esi
       mov      r14d, 1
       or       ebp, edi
       cmove    r11d, r14d
       or       byte  ptr [rcx+0x1C], sil
       vmovsd   xmm3, qword ptr [rdx]
       vinsertps xmm3, xmm3, dword ptr [rdx+0x08], 40
       vmovsd   xmm4, qword ptr [rcx]
       vinsertps xmm4, xmm4, dword ptr [rcx+0x08], 40
       mov      ebp, -1
       xor      r14d, r14d
       test     esi, esi
       cmove    ebp, r14d
       vmovd    xmm5, ebp
       vpbroadcastd xmm5, xmm5
       vandps   xmm3, xmm3, xmm5
       vandnps  xmm4, xmm5, xmm4
       vorps    xmm3, xmm4, xmm3
       vmovsd   qword ptr [rcx], xmm3
       vextractps dword ptr [rcx+0x08], xmm3, 2
       vmovsd   xmm3, qword ptr [r8]
       vinsertps xmm3, xmm3, dword ptr [r8+0x08], 40
       vmovsd   xmm4, qword ptr [rcx+0x0C]
       vinsertps xmm4, xmm4, dword ptr [rcx+0x14], 40
       mov      ebp, -1
 
G_M000_IG14:                ;; offset=0x0635
       test     esi, esi
       cmove    ebp, r14d
       vmovd    xmm5, ebp
       vpbroadcastd xmm5, xmm5
       vandps   xmm3, xmm3, xmm5
       vandnps  xmm4, xmm5, xmm4
       vorps    xmm3, xmm4, xmm3
       vmovsd   qword ptr [rcx+0x0C], xmm3
       vextractps dword ptr [rcx+0x14], xmm3, 2
       mov      rsi, r9
       or       byte  ptr [rsi+0x1C], dil
       vmovsd   xmm3, qword ptr [rdx]
       vinsertps xmm3, xmm3, dword ptr [rdx+0x08], 40
       vmovsd   xmm4, qword ptr [rsi]
       vinsertps xmm4, xmm4, dword ptr [rsi+0x08], 40
       mov      ebp, -1
 
G_M000_IG15:                ;; offset=0x067E
       test     edi, edi
       cmove    ebp, r14d
       vmovd    xmm5, ebp
       vpbroadcastd xmm5, xmm5
       vandps   xmm3, xmm3, xmm5
       vandnps  xmm4, xmm5, xmm4
       vorps    xmm3, xmm4, xmm3
       vmovsd   qword ptr [rsi], xmm3
       vextractps dword ptr [rsi+0x08], xmm3, 2
       vmovsd   xmm3, qword ptr [r8]
       vinsertps xmm3, xmm3, dword ptr [r8+0x08], 40
       vmovsd   xmm4, qword ptr [rsi+0x0C]
       vinsertps xmm4, xmm4, dword ptr [rsi+0x14], 40
       mov      ebp, -1
 
G_M000_IG16:                ;; offset=0x06C1
       test     edi, edi
       cmove    ebp, r14d
       vmovd    xmm5, ebp
       vpbroadcastd xmm5, xmm5
       vandps   xmm3, xmm3, xmm5
       vandnps  xmm4, xmm5, xmm4
       vorps    xmm3, xmm4, xmm3
       vmovsd   qword ptr [rsi+0x0C], xmm3
       vextractps dword ptr [rsi+0x14], xmm3, 2
       mov      rsi, rbx
       or       byte  ptr [rsi+0x1C], r11b
       vmovsd   xmm3, qword ptr [rdx]
       vinsertps xmm3, xmm3, dword ptr [rdx+0x08], 40
       vmovsd   xmm4, qword ptr [rsi]
       vinsertps xmm4, xmm4, dword ptr [rsi+0x08], 40
       mov      edx, -1
 
G_M000_IG17:                ;; offset=0x070A
       test     r11d, r11d
       cmove    edx, r14d
       vmovd    xmm5, edx
       vpbroadcastd xmm5, xmm5
       vandps   xmm3, xmm3, xmm5
       vandnps  xmm4, xmm5, xmm4
       vorps    xmm3, xmm4, xmm3
       vmovsd   qword ptr [rsi], xmm3
       vextractps dword ptr [rsi+0x08], xmm3, 2
       vmovsd   xmm3, qword ptr [r8]
       vinsertps xmm3, xmm3, dword ptr [r8+0x08], 40
       vmovsd   xmm4, qword ptr [rsi+0x0C]
       vinsertps xmm4, xmm4, dword ptr [rsi+0x14], 40
       mov      edx, -1
 
G_M000_IG18:                ;; offset=0x074E
       test     r11d, r11d
       cmove    edx, r14d
       vmovd    xmm5, edx
       vpbroadcastd xmm5, xmm5
       vandps   xmm3, xmm3, xmm5
       vandnps  xmm4, xmm5, xmm4
       vorps    xmm3, xmm4, xmm3
       vmovsd   qword ptr [rsi+0x0C], xmm3
       vextractps dword ptr [rsi+0x14], xmm3, 2
 
G_M000_IG19:                ;; offset=0x0776
       vmovsd   xmm3, qword ptr [rcx+0x20]
       vinsertps xmm3, xmm3, dword ptr [rcx+0x28], 40
       vmovsd   xmm4, qword ptr [rcx]
       vinsertps xmm4, xmm4, dword ptr [rcx+0x08], 40
       vsubps   xmm5, xmm3, xmm4
       vmovsd   xmm6, qword ptr [rcx+0x40]
       vinsertps xmm6, xmm6, dword ptr [rcx+0x48], 40
       vsubps   xmm7, xmm4, xmm6
       vsubps   xmm3, xmm6, xmm3
       vmovaps  xmm8, xmm7
       vmovaps  xmm9, xmm8
       vmovaps  xmm10, xmm5
       vmovaps  xmm11, xmm10
       vpermilps xmm12, xmm11, 9
       vpermilps xmm13, xmm9, 18
       vmovaps  xmmword ptr [rsp], xmm13
       vmulps   xmm14, xmm13, xmm12
       vpermilps xmm11, xmm11, 18
       vmovaps  xmmword ptr [rsp+0x20], xmm11
       vpermilps xmm9, xmm9, 9
       vmovaps  xmmword ptr [rsp+0x10], xmm9
       vmulps   xmm15, xmm9, xmm11
       vsubps   xmm14, xmm14, xmm15
       vmovaps  xmm15, xmm14
       vmulss   xmm2, xmm15, xmm15
       vmovshdup xmm1, xmm14
       vmulss   xmm1, xmm1, xmm1
       vaddss   xmm1, xmm2, xmm1
       vunpckhps xmm2, xmm14, xmm14
       vmulss   xmm2, xmm2, xmm2
       vaddss   xmm1, xmm1, xmm2
       vmovaps  xmm2, xmmword ptr [rsp+0xE0]
       vsubps   xmm4, xmm4, xmm2
       vsubps   xmm6, xmm6, xmm2
       vmovaps  xmm13, xmm4
       vmovaps  xmm9, xmm13
       vpermilps xmm11, xmm9, 18
       vmulps   xmm11, xmm11, xmm12
       vpermilps xmm9, xmm9, 9
       vmulps   xmm9, xmm9, xmmword ptr [rsp+0x20]
       vsubps   xmm9, xmm11, xmm9
       vmovaps  xmm11, xmm6
       vpermilps xmm12, xmm11, 18
       vmulps   xmm12, xmm12, xmmword ptr [rsp+0x10]
       vpermilps xmm11, xmm11, 9
       vmulps   xmm11, xmm11, xmmword ptr [rsp]
       vsubps   xmm11, xmm12, xmm11
       vmovaps  xmmword ptr [rsp+0xC0], xmm11
       vinsertps xmm9, xmm9, xmm9, 56
       vmovaps  xmm12, xmm14
       vinsertps xmm12, xmm12, xmm12, 56
       vmulps   xmm9, xmm9, xmm12
       vpermilps xmm11, xmm9, -11
       vaddps   xmm11, xmm11, xmm9
       vpermilps xmm9, xmm9, -86
       vaddps   xmm9, xmm9, xmm11
       vmovaps  xmm11, xmmword ptr [rsp+0xC0]
       vinsertps xmm11, xmm11, xmm11, 56
       vmulps   xmm11, xmm11, xmm12
       vpermilps xmm0, xmm11, -11
 
G_M000_IG20:                ;; offset=0x08B8
       vaddps   xmm0, xmm0, xmm11
       vpermilps xmm11, xmm11, -86
       vaddps   xmm0, xmm11, xmm0
       vmovss   dword ptr [rsp+0xBC], xmm0
       vsubss   xmm11, xmm1, xmm0
       vsubss   xmm11, xmm11, xmm9
       vmovss   dword ptr [rsp+0xB8], xmm11
       vxorps   xmm11, xmm11, xmm11
       vucomiss xmm11, dword ptr [rsp+0xB8]
       seta     dl
       movzx    rdx, dl
       vucomiss xmm11, xmm0
       seta     r8b
       movzx    r8, r8b
       vmovaps  xmm11, xmm5
       vmovaps  xmm0, xmm5
       vmulss   xmm0, xmm11, xmm0
       vmovshdup xmm11, xmm5
       vmulss   xmm11, xmm11, xmm11
       vaddss   xmm0, xmm0, xmm11
       vunpckhps xmm5, xmm5, xmm5
       vmulss   xmm5, xmm5, xmm5
       vaddss   xmm0, xmm0, xmm5
       vmovss   dword ptr [rsp+0xB4], xmm0
       vmovaps  xmm5, xmm3
       vmovaps  xmm11, xmm3
       vmulss   xmm5, xmm5, xmm11
       vmovshdup xmm11, xmm3
       vmulss   xmm11, xmm11, xmm11
       vaddss   xmm5, xmm5, xmm11
       vunpckhps xmm11, xmm3, xmm3
       vmulss   xmm11, xmm11, xmm11
       vaddss   xmm5, xmm5, xmm11
       vmovaps  xmm11, xmm7
       vmovaps  xmm0, xmm7
       vmulss   xmm0, xmm11, xmm0
       vmovshdup xmm11, xmm7
       vmulss   xmm11, xmm11, xmm11
       vaddss   xmm0, xmm0, xmm11
       vunpckhps xmm7, xmm7, xmm7
       vmulss   xmm7, xmm7, xmm7
       vaddss   xmm0, xmm0, xmm7
       vmovss   xmm11, dword ptr [rsp+0xB4]
       vmaxss   xmm11, xmm11, xmm5
       vmaxss   xmm11, xmm11, xmm0
       vmulss   xmm7, xmm11, dword ptr [reloc @RWD00]
       vucomiss xmm7, xmm1
       setae    r11b
       movzx    r11, r11b
       vmovss   xmm7, dword ptr [reloc @RWD04]
       vucomiss xmm7, xmm11
       seta     sil
       movzx    rsi, sil
       test     esi, esi
       sete     dil
       movzx    rdi, dil
       and      edi, r11d
       vmovsd   xmm7, qword ptr [r10]
       vinsertps xmm7, xmm7, dword ptr [r10+0x08], 40
       vinsertps xmm7, xmm7, xmm7, 56
       vmulps   xmm7, xmm7, xmm12
       vpermilps xmm11, xmm7, -11
       vaddps   xmm11, xmm11, xmm7
       vpermilps xmm7, xmm7, -86
       vaddps   xmm7, xmm7, xmm11
       vxorps   xmm11, xmm11, xmm11
 
G_M000_IG21:                ;; offset=0x09F8
       vucomiss xmm11, xmm7
       seta     r10b
       movzx    r10, r10b
       vxorps   xmm7, xmm15, xmmword ptr [reloc @RWD16]
       mov      ebp, -1
       xor      r14d, r14d
       test     r10d, r10d
       cmove    ebp, r14d
       vmovd    xmm11, ebp
       vandps   xmm7, xmm7, xmm11
       vandnps  xmm11, xmm11, xmm15
       vorps    xmm7, xmm11, xmm7
       vinsertps xmm14, xmm14, xmm7, 0
       vmovshdup xmm7, xmm14
       vxorps   xmm11, xmm7, xmmword ptr [reloc @RWD16]
       mov      ebp, -1
 
G_M000_IG22:                ;; offset=0x0A45
       test     r10d, r10d
       cmove    ebp, r14d
       vmovd    xmm12, ebp
       vandps   xmm11, xmm11, xmm12
       vandnps  xmm7, xmm12, xmm7
       vorps    xmm7, xmm7, xmm11
       vinsertps xmm14, xmm14, xmm7, 16
       vxorps   xmm7, xmm7, xmm7
       vucomiss xmm7, xmm9
       seta     bpl
       movzx    rbp, bpl
       vunpckhps xmm7, xmm14, xmm14
       vxorps   xmm11, xmm7, xmmword ptr [reloc @RWD16]
       mov      r14d, -1
       xor      r15d, r15d
       test     r10d, r10d
       cmove    r14d, r15d
       vmovd    xmm12, r14d
       vandps   xmm11, xmm11, xmm12
       vandnps  xmm7, xmm12, xmm7
       vorps    xmm7, xmm7, xmm11
       vinsertps xmm14, xmm14, xmm7, 32
       vxorps   xmm7, xmm13, xmmword ptr [reloc @RWD16]
       mov      r10d, 1
       vmovss   xmm11, dword ptr [reloc @RWD32]
       vmovss   dword ptr [rcx+0x18], xmm11
 
G_M000_IG23:                ;; offset=0x0AC6
       mov      dword ptr [rcx+0x38], r15d
 
G_M000_IG24:                ;; offset=0x0ACA
       mov      dword ptr [rcx+0x58], r15d
       vmovss   dword ptr [rcx+0x60], xmm11
       vmovaps  xmm12, xmm4
       vmovaps  xmm15, xmm4
       vmulss   xmm12, xmm12, xmm15
       vmovshdup xmm15, xmm4
       vmulss   xmm15, xmm15, xmm15
       vaddss   xmm12, xmm12, xmm15
       vunpckhps xmm4, xmm4, xmm4
       vmulss   xmm4, xmm4, xmm4
       vaddss   xmm4, xmm12, xmm4
       or       edx, ebp
       or       edx, r8d
       vmovss   xmm12, dword ptr [rsp+0xDC]
       vucomiss xmm12, xmm4
       seta     r8b
       movzx    r8, r8b
       test     r8d, esi
       je       SHORT G_M000_IG26
 
G_M000_IG25:                ;; offset=0x0B19
       mov      byte  ptr [rax], 1
 
G_M000_IG26:                ;; offset=0x0B1C
       or       edi, edx
       cmp      byte  ptr [rax], 0
       sete     r8b
       movzx    r8, r8b
       test     edi, r8d
       je       G_M000_IG47
 
G_M000_IG27:                ;; offset=0x0B32
       vmovss   xmm7, dword ptr [rsp+0xB4]
       vdivss   xmm4, xmm11, xmm7
       vmovss   dword ptr [rsp+0xB0], xmm4
       vdivss   xmm15, xmm11, xmm5
       vmovss   dword ptr [rsp+0xAC], xmm15
       vdivss   xmm15, xmm11, xmm0
       vmovss   dword ptr [rsp+0xA8], xmm15
       vmovsd   xmm15, qword ptr [rcx+0x20]
       vinsertps xmm15, xmm15, dword ptr [rcx+0x28], 40
       vsubps   xmm15, xmm15, xmm2
       vmovaps  xmmword ptr [rsp+0x90], xmm15
       vinsertps xmm4, xmm13, xmm13, 56
       vinsertps xmm15, xmm10, xmm10, 56
       vmulps   xmm4, xmm15, xmm4
       vpermilps xmm15, xmm4, -11
       vaddps   xmm15, xmm15, xmm4
       vpermilps xmm4, xmm4, -86
       vaddps   xmm4, xmm4, xmm15
       vmovss   dword ptr [rsp+0x8C], xmm4
       vmovaps  xmm15, xmmword ptr [rsp+0x90]
       vmovaps  xmmword ptr [rsp+0x30], xmm15
       vinsertps xmm4, xmm15, xmm15, 56
       vmovaps  xmm12, xmm3
       vinsertps xmm12, xmm12, xmm12, 56
       vmulps   xmm4, xmm12, xmm4
       vpermilps xmm12, xmm4, -11
       vaddps   xmm12, xmm12, xmm4
       vpermilps xmm4, xmm4, -86
       vaddps   xmm4, xmm4, xmm12
       vmovss   dword ptr [rsp+0x88], xmm4
       vmovaps  xmm12, xmm6
       vinsertps xmm12, xmm12, xmm12, 56
       vinsertps xmm4, xmm8, xmm8, 56
       vmulps   xmm4, xmm4, xmm12
       vpermilps xmm12, xmm4, -11
       vaddps   xmm12, xmm12, xmm4
       vpermilps xmm4, xmm4, -86
       vaddps   xmm4, xmm4, xmm12
       vmovss   dword ptr [rsp+0x84], xmm4
       vmovss   xmm4, dword ptr [rsp+0x8C]
       vxorps   xmm4, xmm4, xmmword ptr [reloc @RWD16]
       vminss   xmm4, xmm7, xmm4
       vxorps   xmm7, xmm7, xmm7
       vmaxss   xmm4, xmm7, xmm4
       vmovss   xmm12, dword ptr [rsp+0x88]
       vxorps   xmm7, xmm12, xmmword ptr [reloc @RWD16]
       vmovaps  xmm12, xmm5
       vminss   xmm7, xmm12, xmm7
       vxorps   xmm12, xmm12, xmm12
       vmaxss   xmm7, xmm12, xmm7
       vmulss   xmm4, xmm4, dword ptr [rsp+0xB0]
       vmovss   dword ptr [rsp+0x80], xmm4
       vmulss   xmm7, xmm7, dword ptr [rsp+0xAC]
       vmovss   dword ptr [rsp+0x7C], xmm7
       vmovss   xmm7, dword ptr [rsp+0x84]
       vxorps   xmm7, xmm7, xmmword ptr [reloc @RWD16]
       vmovaps  xmm12, xmm0
       vminss   xmm7, xmm12, xmm7
 
G_M000_IG28:                ;; offset=0x0C96
       vxorps   xmm12, xmm12, xmm12
       vmaxss   xmm7, xmm12, xmm7
       vmulss   xmm7, xmm7, dword ptr [rsp+0xA8]
       vmovss   dword ptr [rsp+0x78], xmm7
       vbroadcastss xmm12, xmm4
       vmulps   xmm12, xmm12, xmm10
       vmovaps  xmmword ptr [rsp+0x60], xmm12
       vmovaps  xmm12, xmm3
       vbroadcastss xmm4, dword ptr [rsp+0x7C]
       vmulps   xmm4, xmm4, xmm12
       vbroadcastss xmm12, xmm7
       vmulps   xmm12, xmm12, xmm8
       vmovaps  xmmword ptr [rsp+0x50], xmm12
       vmovaps  xmm12, xmmword ptr [rsp+0x60]
       vaddps   xmm12, xmm12, xmm13
       vaddps   xmm4, xmm4, xmm15
       vmovaps  xmm7, xmm6
       vmovaps  xmm15, xmmword ptr [rsp+0x50]
       vaddps   xmm7, xmm15, xmm7
       vmovaps  xmm15, xmm12
       vmovss   dword ptr [rsp+0xFC], xmm15
       vmovaps  xmm15, xmm12
       vmulss   xmm15, xmm15, dword ptr [rsp+0xFC]
       vmovss   dword ptr [rsp+0xFC], xmm15
       vmovshdup xmm15, xmm12
       vmovss   dword ptr [rsp+0xF8], xmm15
       vmovshdup xmm15, xmm12
       vmulss   xmm15, xmm15, dword ptr [rsp+0xF8]
       vaddss   xmm15, xmm15, dword ptr [rsp+0xFC]
       vmovss   dword ptr [rsp+0xFC], xmm15
       vunpckhps xmm15, xmm12, xmm12
       vunpckhps xmm12, xmm12, xmm12
       vmulss   xmm12, xmm15, xmm12
       vaddss   xmm12, xmm12, dword ptr [rsp+0xFC]
       vmovss   dword ptr [rsp+0x4C], xmm12
       vmovaps  xmm15, xmm4
       vmovaps  xmm12, xmm4
       vmulss   xmm12, xmm15, xmm12
       vmovss   dword ptr [rsp+0xFC], xmm12
       vmovshdup xmm15, xmm4
       vmovshdup xmm12, xmm4
       vmulss   xmm12, xmm15, xmm12
       vaddss   xmm12, xmm12, dword ptr [rsp+0xFC]
       vunpckhps xmm15, xmm4, xmm4
       vunpckhps xmm4, xmm4, xmm4
       vmulss   xmm4, xmm15, xmm4
       vaddss   xmm4, xmm12, xmm4
       vmovss   dword ptr [rsp+0x48], xmm4
       vmovaps  xmm12, xmm7
       vmovaps  xmm15, xmm7
       vmulss   xmm12, xmm12, xmm15
       vmovshdup xmm15, xmm7
       vmovshdup xmm4, xmm7
       vmulss   xmm4, xmm15, xmm4
       vaddss   xmm4, xmm12, xmm4
       vunpckhps xmm12, xmm7, xmm7
       vunpckhps xmm7, xmm7, xmm7
       vmulss   xmm7, xmm12, xmm7
       vaddss   xmm4, xmm4, xmm7
       vxorps   xmm7, xmm7, xmm7
       vucomiss xmm0, xmm7
       setnp    r8b
       jp       SHORT G_M000_IG29
       sete     r8b
 
G_M000_IG29:                ;; offset=0x0DEE
       movzx    r8, r8b
       vmovss   xmm12, dword ptr [rsp+0x4C]
       vmovss   xmm0, dword ptr [rsp+0x48]
       vucomiss xmm0, xmm12
       seta     r10b
       movzx    r10, r10b
       vucomiss xmm5, xmm7
       setnp    sil
       jp       SHORT G_M000_IG30
       sete     sil
 
G_M000_IG30:                ;; offset=0x0E19
       movzx    rsi, sil
       or       r10d, esi
       vucomiss xmm4, xmm12
       seta     sil
       movzx    rsi, sil
       or       esi, r8d
       movzx    rsi, sil
       and      r10d, esi
       vucomiss xmm4, xmm0
       seta     sil
       movzx    rsi, sil
       or       r8d, esi
       test     r10d, r10d
       sete     sil
       movzx    rsi, sil
       and      r8d, esi
       mov      esi, -1
       xor      edi, edi
       test     r8d, r8d
       cmove    esi, edi
       vmovd    xmm5, esi
       vandps   xmm0, xmm0, xmm5
       vandnps  xmm4, xmm5, xmm4
       vorps    xmm0, xmm4, xmm0
       mov      esi, -1
 
G_M000_IG31:                ;; offset=0x0E76
       test     r10d, r10d
       cmove    esi, edi
       vmovd    xmm4, esi
       vandps   xmm5, xmm12, xmm4
       vandnps  xmm0, xmm4, xmm0
       vorps    xmm0, xmm0, xmm5
       vmovss   xmm12, dword ptr [rsp+0xDC]
       vucomiss xmm12, xmm0
       jb       SHORT G_M000_IG32
       mov      byte  ptr [rax], 1
 
G_M000_IG32:                ;; offset=0x0E9E
       mov      esi, -1
       xor      edi, edi
       test     r8d, r8d
       cmove    esi, edi
       vmovd    xmm0, esi
       vmovss   xmm4, dword ptr [rsp+0x7C]
       vandps   xmm4, xmm4, xmm0
       vmovss   xmm5, dword ptr [rsp+0x78]
       vandnps  xmm0, xmm0, xmm5
       vorps    xmm0, xmm0, xmm4
       mov      esi, -1
 
G_M000_IG33:                ;; offset=0x0ECC
       test     r10d, r10d
       cmove    esi, edi
       vmovd    xmm4, esi
       vmovss   xmm5, dword ptr [rsp+0x80]
       vandps   xmm5, xmm5, xmm4
       vandnps  xmm0, xmm4, xmm0
       vorps    xmm0, xmm0, xmm5
       mov      esi, -1
 
G_M000_IG34:                ;; offset=0x0EF0
       test     r10d, r10d
       cmove    esi, edi
       vmovd    xmm4, esi
       vpbroadcastd xmm4, xmm4
       vandps   xmm5, xmm10, xmm4
       vandnps  xmm4, xmm4, xmm8
       vorps    xmm4, xmm4, xmm5
       mov      esi, -1
 
G_M000_IG35:                ;; offset=0x0F11
       test     r8d, r8d
       cmove    esi, edi
       vmovd    xmm5, esi
       vpbroadcastd xmm5, xmm5
       vandps   xmm3, xmm3, xmm5
       vandnps  xmm4, xmm5, xmm4
       vorps    xmm4, xmm4, xmm3
       mov      esi, -1
 
G_M000_IG36:                ;; offset=0x0F31
       test     r10d, r10d
       cmove    esi, edi
       vmovd    xmm3, esi
       vpbroadcastd xmm3, xmm3
       vandps   xmm5, xmm13, xmm3
       vandnps  xmm3, xmm3, xmm6
       vorps    xmm3, xmm3, xmm5
       mov      esi, -1
 
G_M000_IG37:                ;; offset=0x0F51
       test     r8d, r8d
       cmove    esi, edi
       vmovd    xmm5, esi
       vpbroadcastd xmm5, xmm5
       vmovaps  xmm15, xmmword ptr [rsp+0x30]
       vandps   xmm6, xmm15, xmm5
       vandnps  xmm3, xmm5, xmm3
       vorps    xmm3, xmm3, xmm6
       vxorps   xmm5, xmm0, xmmword ptr [reloc @RWD16]
       vbroadcastss xmm5, xmm5
       vmulps   xmm4, xmm5, xmm4
       vucomiss xmm0, xmm11
       setnp    sil
       jp       SHORT G_M000_IG38
       sete     sil
 
G_M000_IG38:                ;; offset=0x0F92
       movzx    rsi, sil
       vxorps   xmm5, xmm5, xmm5
       vucomiss xmm0, xmm5
       setnp    dil
       jp       SHORT G_M000_IG39
       sete     dil
 
G_M000_IG39:                ;; offset=0x0FA8
       movzx    rdi, dil
       vsubps   xmm7, xmm4, xmm3
       mov      ebp, -1
       xor      r14d, r14d
       test     esi, esi
       cmove    ebp, r14d
       mov      r14d, -1
       xor      r15d, r15d
       test     edi, edi
       cmove    r14d, r15d
       mov      r15d, r14d
       and      r15d, 1
       mov      r13d, 3
       andn     r13d, ebp, r13d
       and      ebp, 2
       or       ebp, r13d
       andn     ebp, r14d, ebp
       or       ebp, r15d
       mov      r14d, -1
       xor      r15d, r15d
       test     esi, esi
       cmove    r14d, r15d
       mov      r15d, -1
       xor      r13d, r13d
       test     edi, edi
       cmove    r15d, r13d
       mov      r13d, r15d
       and      r13d, 2
       mov      r12d, 6
       andn     r12d, r14d, r12d
       and      r14d, 4
       or       r14d, r12d
       andn     r14d, r15d, r14d
       or       r14d, r13d
       mov      r15d, -1
       xor      r13d, r13d
       test     esi, esi
       cmove    r15d, r13d
       mov      esi, -1
 
G_M000_IG40:                ;; offset=0x1040
       test     edi, edi
       cmove    esi, r13d
       mov      edi, esi
       and      edi, 4
       mov      r13d, 5
       andn     r13d, r15d, r13d
       and      r15d, 1
       or       r15d, r13d
       andn     esi, esi, r15d
       or       esi, edi
       mov      edi, -1
       xor      r15d, r15d
       test     r8d, r8d
       cmove    edi, r15d
       mov      r15d, -1
       xor      r13d, r13d
       test     r10d, r10d
       cmove    r15d, r13d
       and      r14d, edi
       andn     esi, edi, esi
       or       esi, r14d
       andn     esi, r15d, esi
       and      ebp, r15d
       or       ebp, esi
       vsubss   xmm3, xmm11, xmm0
       mov      esi, -1
 
G_M000_IG41:                ;; offset=0x10A1
       test     r8d, r8d
       cmove    esi, r13d
       vmovd    xmm4, esi
       vxorps   xmm5, xmm5, xmm5
       vandps   xmm5, xmm5, xmm4
       vmovaps  xmm6, xmm0
       vandnps  xmm4, xmm4, xmm6
       vorps    xmm4, xmm4, xmm5
       mov      esi, -1
 
G_M000_IG42:                ;; offset=0x10C5
       test     r10d, r10d
       cmove    esi, r13d
       vmovd    xmm5, esi
       vmovaps  xmm6, xmm3
       vandps   xmm6, xmm6, xmm5
       vandnps  xmm4, xmm5, xmm4
       vorps    xmm4, xmm4, xmm6
       vmovss   dword ptr [rcx+0x18], xmm4
       mov      esi, -1
 
G_M000_IG43:                ;; offset=0x10EA
       test     r8d, r8d
       cmove    esi, r13d
       vmovd    xmm4, esi
       vmovaps  xmm5, xmm3
       vandps   xmm5, xmm5, xmm4
       vxorps   xmm6, xmm6, xmm6
       vandnps  xmm4, xmm4, xmm6
       vorps    xmm4, xmm4, xmm5
       mov      esi, -1
 
G_M000_IG44:                ;; offset=0x110E
       test     r10d, r10d
       cmove    esi, r13d
       vmovd    xmm5, esi
       vmovaps  xmm6, xmm0
       vandps   xmm6, xmm6, xmm5
       vandnps  xmm4, xmm5, xmm4
       vorps    xmm4, xmm4, xmm6
       vmovss   dword ptr [r9+0x18], xmm4
       mov      r9d, -1
 
G_M000_IG45:                ;; offset=0x1135
       test     r8d, r8d
       cmove    r9d, r13d
       vmovd    xmm4, r9d
       vandps   xmm0, xmm0, xmm4
       vandnps  xmm3, xmm4, xmm3
       vorps    xmm0, xmm3, xmm0
       mov      r8d, -1
 
G_M000_IG46:                ;; offset=0x1153
       test     r10d, r10d
       cmove    r8d, r13d
       vmovd    xmm3, r8d
       vxorps   xmm4, xmm4, xmm4
       vandps   xmm4, xmm4, xmm3
       vandnps  xmm0, xmm3, xmm0
       vorps    xmm0, xmm0, xmm4
       vmovss   dword ptr [rbx+0x18], xmm0
       mov      r10d, ebp
 
G_M000_IG47:                ;; offset=0x1177
       test     edx, edx
       sete     dl
       movzx    rdx, dl
       test     r11d, r11d
       sete     r8b
       movzx    r8, r8b
       and      edx, r8d
       cmp      byte  ptr [rax], 0
       sete     r8b
       movzx    r8, r8b
       and      edx, r8d
       je       SHORT G_M000_IG50
 
G_M000_IG48:                ;; offset=0x119D
       vinsertps xmm0, xmm13, xmm13, 56
       vmovaps  xmm3, xmm14
       vinsertps xmm3, xmm3, xmm3, 56
       vmulps   xmm0, xmm3, xmm0
       vpermilps xmm3, xmm0, -11
       vaddps   xmm3, xmm3, xmm0
       vpermilps xmm0, xmm0, -86
       vaddps   xmm0, xmm0, xmm3
       vmulss   xmm0, xmm0, xmm0
       vmulss   xmm3, xmm1, dword ptr [rsp+0xDC]
       vucomiss xmm3, xmm0
       jbe      SHORT G_M000_IG49
       mov      byte  ptr [rax], 1
 
G_M000_IG49:                ;; offset=0x11DC
       vmovaps  xmm7, xmm14
       mov      r10d, 7
       vmovss   xmm0, dword ptr [rsp+0xB8]
       vmovss   dword ptr [rcx+0x18], xmm0
       vmovss   xmm0, dword ptr [rsp+0xBC]
       vmovss   dword ptr [rcx+0x38], xmm0
       vmovss   dword ptr [rcx+0x58], xmm9
       vmovss   dword ptr [rcx+0x60], xmm1
 
G_M000_IG50:                ;; offset=0x120D
       mov      r8d, r10d
       and      r8d, 1
       setg     r8b
       mov      byte  ptr [rcx+0x1C], r8b
       mov      r8d, r10d
       and      r8d, 2
       setg     r8b
       mov      byte  ptr [rcx+0x3C], r8b
       and      r10d, 4
       setg     r8b
       mov      byte  ptr [rcx+0x5C], r8b
       cmp      byte  ptr [rax], 0
       jne      G_M000_IG52
 
G_M000_IG51:                ;; offset=0x1240
       vmovaps  xmm0, xmm7
       vmulps   xmm0, xmm0, xmmword ptr [reloc @RWD48]
       vaddps   xmm0, xmm0, xmm2
       vxorps   xmm1, xmm1, xmm1
       vucomiss xmm1, dword ptr [rsp+0x220]
       setae    al
       movzx    rax, al
       or       al, dl
       mov      ecx, -1
       xor      edx, edx
       test     al, al
       cmove    ecx, edx
       vmovd    xmm1, ecx
       vpbroadcastd xmm1, xmm1
       vandps   xmm2, xmm7, xmm1
       vandnps  xmm0, xmm1, xmm0
       vorps    xmm7, xmm0, xmm2
       vmovaps  xmm0, xmm7
       vmovaps  xmm1, xmm7
       vmulss   xmm0, xmm0, xmm1
       vmovshdup xmm1, xmm7
       vmovshdup xmm2, xmm7
       vmulss   xmm1, xmm1, xmm2
       vaddss   xmm0, xmm0, xmm1
       vunpckhps xmm1, xmm7, xmm7
       vunpckhps xmm2, xmm7, xmm7
       vmulss   xmm1, xmm1, xmm2
       vaddss   xmm0, xmm0, xmm1
       vsqrtss  xmm0, xmm0, xmm0
       vdivss   xmm0, xmm11, xmm0
       vbroadcastss xmm0, xmm0
       vmovaps  xmm1, xmm7
       vmulps   xmm0, xmm0, xmm1
       mov      rax, bword ptr [rsp+0x230]
       vmovsd   qword ptr [rax], xmm0
       vextractps dword ptr [rax+0x08], xmm0, 2
 
G_M000_IG52:                ;; offset=0x12DA
       vmovaps  xmm6, xmmword ptr [rsp+0x190]
       vmovaps  xmm7, xmmword ptr [rsp+0x180]
       vmovaps  xmm8, xmmword ptr [rsp+0x170]
       vmovaps  xmm9, xmmword ptr [rsp+0x160]
       vmovaps  xmm10, xmmword ptr [rsp+0x150]
       vmovaps  xmm11, xmmword ptr [rsp+0x140]
       vmovaps  xmm12, xmmword ptr [rsp+0x130]
       vmovaps  xmm13, xmmword ptr [rsp+0x120]
       vmovaps  xmm14, xmmword ptr [rsp+0x110]
       vmovaps  xmm15, xmmword ptr [rsp+0x100]
       add      rsp, 424
       pop      rbx
       pop      rbp
       pop      rsi
       pop      rdi
       pop      r12
       pop      r13
       pop      r14
       pop      r15
       ret      
 
RWD00  	dd	2EDBE6FFh		;     1e-10
RWD04  	dd	283424DCh		;     1e-14
RWD08  	dd	00000000h, 00000000h
RWD16  	dq	8000000080000000h, 8000000080000000h
RWD32  	dd	3F800000h		;         1
RWD36  	dd	00000000h, 00000000h, 00000000h
RWD48  	dq	4080000040800000h, 4080000040800000h

; Total bytes of code 4936
