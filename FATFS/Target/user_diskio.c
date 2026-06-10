/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * @file    user_diskio.c
  * @brief   FatFS diskio driver — SD card over SPI1 + PA2 chip-select.
  *          Supports SDSC (CMD17/24 byte-addressed) and SDHC/SDXC (sector-addressed).
  ******************************************************************************
  */
 /* USER CODE END Header */

#ifdef USE_OBSOLETE_USER_CODE_SECTION_0
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
#endif

/* USER CODE BEGIN DECL */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "ff_gen_drv.h"
#include "main.h"

/* ── Hardware bindings ───────────────────────────────────────────────────────── */
extern SPI_HandleTypeDef hspi1;
#define SD_CS_LOW()   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET)
#define SD_CS_HIGH()  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET)
#define SPI_TO        500U

/* ── Card type ───────────────────────────────────────────────────────────────── */
#define CT_UNKNOWN  0x00
#define CT_SD1      0x02
#define CT_SD2      0x04
#define CT_SDHC     0x08
#define CT_MMC      0x01

/* Private variables ---------------------------------------------------------*/
static volatile DSTATUS Stat     = STA_NOINIT;
static uint8_t          CardType = CT_UNKNOWN;

/* USER CODE END DECL */

/* Private function prototypes -----------------------------------------------*/
DSTATUS USER_initialize (BYTE pdrv);
DSTATUS USER_status (BYTE pdrv);
DRESULT USER_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
  DRESULT USER_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  DRESULT USER_ioctl (BYTE pdrv, BYTE cmd, void *buff);
#endif /* _USE_IOCTL == 1 */

Diskio_drvTypeDef  USER_Driver =
{
  USER_initialize,
  USER_status,
  USER_read,
#if  _USE_WRITE
  USER_write,
#endif
#if  _USE_IOCTL == 1
  USER_ioctl,
#endif
};

/* ── SPI primitives ──────────────────────────────────────────────────────────── */

static uint8_t spi_byte(uint8_t tx) {
    uint8_t rx;
    HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, SPI_TO);
    return rx;
}

static void spi_recv(uint8_t *buf, uint16_t n) {
    uint8_t ff = 0xFF;
    for (uint16_t i = 0; i < n; i++)
        HAL_SPI_TransmitReceive(&hspi1, &ff, &buf[i], 1, SPI_TO);
}

static void spi_send(const uint8_t *buf, uint16_t n) {
    uint8_t dummy;
    for (uint16_t i = 0; i < n; i++)
        HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&buf[i], &dummy, 1, SPI_TO);
}

static uint8_t wait_not_busy(uint32_t ms) {
    uint32_t t = HAL_GetTick();
    uint8_t  d;
    do { d = spi_byte(0xFF); } while (d == 0x00 && (HAL_GetTick() - t) < ms);
    return d;
}

/* ── SD command sender ───────────────────────────────────────────────────────── */

static uint8_t send_cmd(uint8_t cmd, uint32_t arg);

static uint8_t send_cmd(uint8_t cmd, uint32_t arg) {
    uint8_t r1, crc = 0x01;

    if (cmd & 0x80) {
        cmd &= 0x7F;
        r1 = send_cmd(55, 0);
        if (r1 > 1) return r1;
    }

    SD_CS_HIGH(); spi_byte(0xFF);
    SD_CS_LOW();  spi_byte(0xFF);

    if (cmd == 0)  crc = 0x95;
    if (cmd == 8)  crc = 0x87;

    spi_byte(cmd | 0x40);
    spi_byte((uint8_t)(arg >> 24));
    spi_byte((uint8_t)(arg >> 16));
    spi_byte((uint8_t)(arg >>  8));
    spi_byte((uint8_t)(arg      ));
    spi_byte(crc);

    if (cmd == 12) spi_byte(0xFF);

    uint8_t n = 8;
    do { r1 = spi_byte(0xFF); } while ((r1 & 0x80) && --n);
    return r1;
}

/* Private functions ---------------------------------------------------------*/

DSTATUS USER_initialize (BYTE pdrv)
{
  /* USER CODE BEGIN INIT */
    (void)pdrv;
    CardType = CT_UNKNOWN;
    Stat     = STA_NOINIT;

    SD_CS_HIGH();
    for (uint8_t i = 0; i < 10; i++) spi_byte(0xFF);

    uint8_t r0 = send_cmd(0, 0);
    printf("[SD] CMD0=0x%02X\n", r0);
    if (r0 != 0x01) goto done;

    uint8_t r8 = send_cmd(8, 0x000001AA);
    printf("[SD] CMD8=0x%02X\n", r8);
    if (r8 == 0x01) {
        uint8_t r7[4];
        for (uint8_t i = 0; i < 4; i++) r7[i] = spi_byte(0xFF);
        printf("[SD] R7=%02X %02X %02X %02X\n", r7[0], r7[1], r7[2], r7[3]);

        if (r7[2] == 0x01 && r7[3] == 0xAA) {
            uint32_t t = HAL_GetTick();
            uint8_t r1;
            do { r1 = send_cmd(0x80 | 41, 0x40000000); }
            while (r1 && (HAL_GetTick() - t) < 1000);
            printf("[SD] ACMD41=0x%02X\n", r1);

            if (!r1) {
                if (send_cmd(58, 0) == 0x00) {
                    uint8_t ocr[4];
                    for (uint8_t i = 0; i < 4; i++) ocr[i] = spi_byte(0xFF);
                    printf("[SD] OCR=%02X %02X %02X %02X\n", ocr[0], ocr[1], ocr[2], ocr[3]);
                    CardType = (ocr[0] & 0x40) ? CT_SDHC : CT_SD2;
                }
            }
        }
    } else {
        uint8_t ty = (send_cmd(0x80 | 41, 0) <= 1) ? CT_SD1 : CT_MMC;
        uint32_t t = HAL_GetTick();
        uint8_t r1;
        do {
            r1 = (ty == CT_SD1) ? send_cmd(0x80 | 41, 0) : send_cmd(1, 0);
        } while (r1 && (HAL_GetTick() - t) < 1000);
        printf("[SD] v1/MMC ACMD41=0x%02X\n", r1);

        if (r1 == 0x00) {
            if (send_cmd(16, 512) == 0x00) CardType = ty;
        }
    }

done:
    printf("[SD] CardType=0x%02X\n", CardType);
    SD_CS_HIGH();
    spi_byte(0xFF);
    if (CardType != CT_UNKNOWN) Stat &= ~STA_NOINIT;
    return Stat;
  /* USER CODE END INIT */
}

DSTATUS USER_status (BYTE pdrv)
{
  /* USER CODE BEGIN STATUS */
    (void)pdrv;
    return Stat;
  /* USER CODE END STATUS */
}

DRESULT USER_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count)
{
  /* USER CODE BEGIN READ */
    (void)pdrv;
    if (Stat & STA_NOINIT) return RES_NOTRDY;
    if (!(CardType & CT_SDHC)) sector *= 512;

    DRESULT res = RES_ERROR;
    uint8_t token;

    if (count == 1) {
        if (send_cmd(17, sector) == 0x00) {
            uint32_t t = HAL_GetTick();
            do { token = spi_byte(0xFF); } while (token == 0xFF && (HAL_GetTick() - t) < 200);
            if (token == 0xFE) {
                spi_recv(buff, 512);
                spi_byte(0xFF); spi_byte(0xFF);
                res = RES_OK;
            }
        }
    } else {
        if (send_cmd(18, sector) == 0x00) {
            UINT remaining = count;
            do {
                uint32_t t = HAL_GetTick();
                do { token = spi_byte(0xFF); } while (token == 0xFF && (HAL_GetTick() - t) < 200);
                if (token != 0xFE) break;
                spi_recv(buff, 512);
                spi_byte(0xFF); spi_byte(0xFF);
                buff += 512;
            } while (--remaining);
            send_cmd(12, 0);
            if (!remaining) res = RES_OK;
        }
    }

    SD_CS_HIGH();
    spi_byte(0xFF);
    return res;
  /* USER CODE END READ */
}

#if _USE_WRITE == 1
DRESULT USER_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count)
{
  /* USER CODE BEGIN WRITE */
    (void)pdrv;
    if (Stat & STA_NOINIT) return RES_NOTRDY;
    if (!(CardType & CT_SDHC)) sector *= 512;

    DRESULT res = RES_ERROR;

    if (count == 1) {
        if (send_cmd(24, sector) == 0x00) {
            spi_byte(0xFE);
            spi_send(buff, 512);
            spi_byte(0xFF); spi_byte(0xFF);
            uint8_t resp = spi_byte(0xFF) & 0x1F;
            if (resp == 0x05 && wait_not_busy(500) != 0x00)
                res = RES_OK;
        }
    } else {
        if (send_cmd(25, sector) == 0x00) {
            UINT remaining = count;
            do {
                spi_byte(0xFC);
                spi_send(buff, 512);
                spi_byte(0xFF); spi_byte(0xFF);
                uint8_t resp = spi_byte(0xFF) & 0x1F;
                if (resp != 0x05) break;
                wait_not_busy(500);
                buff += 512;
            } while (--remaining);
            spi_byte(0xFD);
            spi_byte(0xFF);
            wait_not_busy(500);
            if (!remaining) res = RES_OK;
        }
    }

    SD_CS_HIGH();
    spi_byte(0xFF);
    return res;
  /* USER CODE END WRITE */
}
#endif /* _USE_WRITE == 1 */

#if _USE_IOCTL == 1
DRESULT USER_ioctl (BYTE pdrv, BYTE cmd, void *buff)
{
  /* USER CODE BEGIN IOCTL */
    (void)pdrv;
    if (Stat & STA_NOINIT) return RES_NOTRDY;

    DRESULT res = RES_ERROR;

    switch (cmd) {
    case CTRL_SYNC:
        SD_CS_LOW();
        if (wait_not_busy(500) != 0x00) res = RES_OK;
        break;
    case GET_SECTOR_SIZE:
        *(WORD *)buff = 512;
        res = RES_OK;
        break;
    case GET_BLOCK_SIZE:
        *(DWORD *)buff = 1;
        res = RES_OK;
        break;
    case GET_SECTOR_COUNT: {
        uint8_t csd[16], token;
        if (send_cmd(9, 0) == 0x00) {
            uint32_t t = HAL_GetTick();
            do { token = spi_byte(0xFF); } while (token == 0xFF && (HAL_GetTick() - t) < 200);
            if (token == 0xFE) {
                for (uint8_t i = 0; i < 16; i++) csd[i] = spi_byte(0xFF);
                spi_byte(0xFF); spi_byte(0xFF);
                DWORD sectors;
                if ((csd[0] >> 6) == 1) {
                    uint32_t csize = ((uint32_t)(csd[7] & 0x3F) << 16)
                                   | ((uint32_t)csd[8] << 8) | csd[9];
                    sectors = (csize + 1) << 10;
                } else {
                    uint32_t csize  = ((uint32_t)(csd[6] & 0x03) << 10)
                                    | ((uint32_t)csd[7] << 2) | (csd[8] >> 6);
                    uint32_t cmult  = ((uint32_t)(csd[9] & 0x03) << 1) | (csd[10] >> 7);
                    uint32_t blklen = 1UL << (csd[5] & 0x0F);
                    sectors = (csize + 1) << (cmult + 2);
                    sectors *= blklen / 512;
                }
                *(DWORD *)buff = sectors;
                res = RES_OK;
            }
        }
        break;
    }
    default:
        res = RES_PARERR;
        break;
    }

    SD_CS_HIGH();
    spi_byte(0xFF);
    return res;
  /* USER CODE END IOCTL */
}
#endif /* _USE_IOCTL == 1 */
