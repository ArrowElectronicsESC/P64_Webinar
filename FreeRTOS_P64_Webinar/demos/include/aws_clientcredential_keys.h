#ifndef AWS_CLIENT_CREDENTIAL_KEYS_H
#define AWS_CLIENT_CREDENTIAL_KEYS_H

#include <stdint.h>

/*
 * PEM-encoded client certificate.
 *
 * Must include the PEM header and footer:
 * "-----BEGIN CERTIFICATE-----\n"\
 * "...base64 data...\n"\
 * "-----END CERTIFICATE-----"
 */
#define keyCLIENT_CERTIFICATE_PEM \
"-----BEGIN CERTIFICATE-----\n"\
"MIIDWTCCAkGgAwIBAgIUdbeOIvzgNxczcqAl33ZPLBQa/WkwDQYJKoZIhvcNAQEL\n"\
"BQAwTTFLMEkGA1UECwxCQW1hem9uIFdlYiBTZXJ2aWNlcyBPPUFtYXpvbi5jb20g\n"\
"SW5jLiBMPVNlYXR0bGUgU1Q9V2FzaGluZ3RvbiBDPVVTMB4XDTIwMTExMjE2MjAy\n"\
"NVoXDTQ5MTIzMTIzNTk1OVowHjEcMBoGA1UEAwwTQVdTIElvVCBDZXJ0aWZpY2F0\n"\
"ZTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAKEumbv8ikWpvl9AcAzQ\n"\
"8fAVgb53yYG1Vrs9YeTnaMQTJZWYDUxpcTPpHTZC6gRX30uoXhoH501Setwus30x\n"\
"rg/ISyKS7sCStsn2bp6NKOUdui+564SeWaI5fTIy2gPQBt8d7VzX1cdOERleUBZD\n"\
"fC8yOTC+8NXiwigrgzDUW2J2YeYhCkWyBdDrEWVUw34dUiFK9Yr3PnF+ATqcRymV\n"\
"nsnunXNLGSuebUpmSs8Um8vxMReB4N+21bEPSkFVPlX4g7jGQpXVXvt3XlmaiSTu\n"\
"A2cgvB3eJ0eQk8LWP9Iv2D0qmUsyh1oHkroDmYJOE0PXPORpq0Yi8f1DvwR603ip\n"\
"j8UCAwEAAaNgMF4wHwYDVR0jBBgwFoAUiY9PZH9qjFWOvulxUsISAblzr0MwHQYD\n"\
"VR0OBBYEFImfhq3t0g4oZQ3nwjT3MCQikr0oMAwGA1UdEwEB/wQCMAAwDgYDVR0P\n"\
"AQH/BAQDAgeAMA0GCSqGSIb3DQEBCwUAA4IBAQCO8w9NyyXCCv5pjFFp6C9pp2BW\n"\
"+qDaNgXyU4RJXyn/lZBaWSeEh4aMpDeZiyxtb1bxxmPRM3jvJgDjEpNc32ObeLc4\n"\
"gO7Iaq+6+0xebx2gKWLejhvi45c6w2vL/xZYw82JD5ZpnIHV+HlQRZOklnnWNPlv\n"\
"CslQ5L8qLoAbPbV116M/tqksb/oOY9xhu393weXJUJTsRnIgEZmZXxWckVgt/0wv\n"\
"866KuNv6tDQXj/4zZEYad3zhjtPTY3wTX7PNEOyvd6X579zZXgW/6ZpLknUDNeZX\n"\
"yQlaFzBHuXyMrz16MCRQp645yRvL+JvpNoVI6B2IxskARTPl5JievDqrNC40\n"\
"-----END CERTIFICATE-----"

/*
 * PEM-encoded client private key.
 *
 * Must include the PEM header and footer:
 * "-----BEGIN RSA PRIVATE KEY-----\n"\
 * "...base64 data...\n"\
 * "-----END RSA PRIVATE KEY-----"
 */
#define keyCLIENT_PRIVATE_KEY_PEM \
"-----BEGIN RSA PRIVATE KEY-----\n"\
"MIIEowIBAAKCAQEAoS6Zu/yKRam+X0BwDNDx8BWBvnfJgbVWuz1h5OdoxBMllZgN\n"\
"TGlxM+kdNkLqBFffS6heGgfnTVJ63C6zfTGuD8hLIpLuwJK2yfZuno0o5R26L7nr\n"\
"hJ5Zojl9MjLaA9AG3x3tXNfVx04RGV5QFkN8LzI5ML7w1eLCKCuDMNRbYnZh5iEK\n"\
"RbIF0OsRZVTDfh1SIUr1ivc+cX4BOpxHKZWeye6dc0sZK55tSmZKzxSby/ExF4Hg\n"\
"37bVsQ9KQVU+VfiDuMZCldVe+3deWZqJJO4DZyC8Hd4nR5CTwtY/0i/YPSqZSzKH\n"\
"WgeSugOZgk4TQ9c85GmrRiLx/UO/BHrTeKmPxQIDAQABAoIBAB20ZuS8QAdipxEK\n"\
"4Rubjlr+u7CMhvh+oQNU5qu28hngpXaSVTEV5aT5e0a6wkIWp7Jbzyvg9QHNB5N4\n"\
"j9+svjVuZlpy1clrwIAsCGrz/OtstRmCnCMOw9of2jDcKk21ku6fp1UK+f0HKCOS\n"\
"vBO+UC6qhNu09NGWPs4EZuMi38yfVcAX07eKScz8uIbkZQToe5Qz+612C4B4TDSO\n"\
"tFsE7YxnI/RqQWdSperQ1NUGbuKG0YVVOKdFcyTNBy+zi6XGJ7DDMzuqS8sX8lAP\n"\
"qT2uyAnai+bQ9mzNHIGjpCzKMsLmxIMA+bM5a9kiStEkqolPLWt+Olr0XEahIuRu\n"\
"2EXtoUECgYEAy04P322NxxH2r8UBHX5QOAWPaPC4mo+wzMljOjfEpPCKEB/64ZnE\n"\
"WL8Ep9jwaSUledjRFpEd2ClSiUTW9BRAsQNPlIuGG0Ib11jtTv5pyrnwkGqx4Fjl\n"\
"YsuPPfr3lPPpEilmfP5L+ttWUddS0tdTd39rfGu8inQzGoIFXfzuNH0CgYEAyvWM\n"\
"ZwK+mutpbvzY192v139Rku46NgklET1RG//SEOyU0/VewtVhPUBVTucmiAHF/fM3\n"\
"+dEo+/TZMtVZzhuHIIT93X//U4yDYzlwXkgW/EF1vV6KvZcOW9mO3TSkU8ueYHnl\n"\
"i4UVL3Lfjo7NZCj94msP8reqoMjrIvixeEPHEukCgYEAvx6mS0ro8lBP0FLpxnkd\n"\
"Hl0TwPozAJG+CfX0AyG0Cb5ae+gkZ/qumyMPVF7bOm0COvZ5MLbBEJlgDs0ZKaD0\n"\
"Y7OSFznGm09dbucZR55hg4Oiw5CZoSbQlTx5bzHDl1JoM3uZ7jz83lldX1iFctGb\n"\
"lIXDXNbHCn1MIUpDk2SX5d0CgYAia/DLC7kd9y0jiLiSK+QFzW8dV1ACGBz0Roz6\n"\
"DbIn8qfvJt9SdfQO+lk9j5CGYp6cymJE+UdVLBqmN28KuLp7NnMD3RCFsAAclQJM\n"\
"yzcywztfyagILZVe2JAVTjdMiu0qeaIHvqNwL7BgsjF9ekJM41GKHcJPkKPog3s9\n"\
"5Wje+QKBgBquOdOI6CFr7N9EFin9d1c2DYiBKn1FfhXCg363S2rFYVtHSBpSlUPb\n"\
"usjiWOX4qOv52+879cIWQioxbNEg6qqb2l8wipB2Q+jwpJd/um+zzLZCvNxUXpeq\n"\
"R8GGQ5vVMaeMmlaDnNm6fCLP9yU4LvISznNfyBsauTmu+L3VBsJY\n"\
"-----END RSA PRIVATE KEY-----"

/*
 * PEM-encoded Just-in-Time Registration (JITR) certificate (optional).
 *
 * If used, must include the PEM header and footer:
 * "-----BEGIN CERTIFICATE-----\n"\
 * "...base64 data...\n"\
 * "-----END CERTIFICATE-----"
 */
#define keyJITR_DEVICE_CERTIFICATE_AUTHORITY_PEM  ""


#endif /* AWS_CLIENT_CREDENTIAL_KEYS_H */
