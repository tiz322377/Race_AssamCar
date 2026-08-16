//
// Created by Administrator on 25-3-2.
//

#include "FDCan.hpp"

#ifdef HAL_FDCAN_MODULE_ENABLED

static uint32_t GetCANLength(const uint8_t _len) {
    switch (_len) {
        case 0: return FDCAN_DLC_BYTES_0;
        case 1: return FDCAN_DLC_BYTES_1;
        case 2: return FDCAN_DLC_BYTES_2;
        case 3: return FDCAN_DLC_BYTES_3;
        case 4: return FDCAN_DLC_BYTES_4;
        case 5: return FDCAN_DLC_BYTES_5;
        case 6: return FDCAN_DLC_BYTES_6;
        case 7: return FDCAN_DLC_BYTES_7;
        case 8: return FDCAN_DLC_BYTES_8;
        default: return 0;
    }
}

void Peripheral::FDCan<Peripheral::Classic>::Transmit(const uint8_t _addr, const uint8_t *_data,
                                                      const uint16_t _len) const {
    FDCAN_TxHeaderTypeDef txHeader = {0};

    // 配置扩展帧参数
    txHeader.Identifier = (static_cast<uint32_t>(_data[0]) << 8); // 扩展ID（29位）
    txHeader.IdType = FDCAN_EXTENDED_ID; // 扩展标识符
    txHeader.TxFrameType = FDCAN_DATA_FRAME; // 数据帧
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF; // 经典模式关闭比特率切换
    txHeader.FDFormat = FDCAN_CLASSIC_CAN; // 经典CAN模式
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker = 0;

    if (_len < 8) {
        txHeader.DataLength = _len;
        HAL_FDCAN_AddMessageToTxFifoQ(fdcan, &txHeader, _data);
        return;
    }

    uint16_t sentLength = 0;
    uint16_t packets = 0;

    while (sentLength < _len) {
        if (_len - sentLength >= 8) {
            txHeader.Identifier = static_cast<uint32_t>(_addr) << 8 | packets;
            txHeader.DataLength = 8;
            HAL_FDCAN_AddMessageToTxFifoQ(fdcan, &txHeader, _data + sentLength);
            sentLength += 8;
        } else {
            txHeader.Identifier = static_cast<uint32_t>(_addr) << 8 | packets;
            txHeader.DataLength = GetCANLength(_len - sentLength);
            HAL_FDCAN_AddMessageToTxFifoQ(fdcan, &txHeader, _data + sentLength);
            sentLength = _len;
        }
        packets++;
    }
}

FDCAN_RxHeaderTypeDef Peripheral::FDCan<Peripheral::Classic>::Receive(uint8_t *_data) const {
    if (HAL_FDCAN_GetRxFifoFillLevel(fdcan, FDCAN_RX_FIFO0) != 0) {
        FDCAN_RxHeaderTypeDef RxHeader;
        if (HAL_FDCAN_GetRxMessage(fdcan, FDCAN_RX_FIFO0, &RxHeader, _data) == HAL_OK) {
            return RxHeader;
        }
    }
    if (HAL_FDCAN_GetRxFifoFillLevel(fdcan, FDCAN_RX_FIFO1) != 0) {
        FDCAN_RxHeaderTypeDef RxHeader;
        if (HAL_FDCAN_GetRxMessage(fdcan, FDCAN_RX_FIFO1, &RxHeader, _data) == HAL_OK) {
            return RxHeader;
        }
    }
    return {};
}

bool Peripheral::FDCan<Peripheral::Classic>::Verify() const {
    FDCAN_TxHeaderTypeDef txHeader = {0};

    // 配置扩展帧参数
    txHeader.Identifier = 1; // 扩展ID（29位）
    txHeader.IdType = FDCAN_EXTENDED_ID; // 扩展标识符
    txHeader.TxFrameType = FDCAN_DATA_FRAME; // 数据帧
    txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    txHeader.BitRateSwitch = FDCAN_BRS_OFF; // 经典模式关闭比特率切换
    txHeader.FDFormat = FDCAN_CLASSIC_CAN; // 经典CAN模式
    txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    txHeader.MessageMarker = 0;
    txHeader.DataLength = 8;

    uint8_t data[8]{0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};

    HAL_FDCAN_AddMessageToTxFifoQ(fdcan, &txHeader, data);

    if (HAL_FDCAN_GetRxFifoFillLevel(fdcan, FDCAN_RX_FIFO0) != 0) {
        FDCAN_RxHeaderTypeDef RxHeader;
        if (HAL_FDCAN_GetRxMessage(fdcan, FDCAN_RX_FIFO0, &RxHeader, data) == HAL_OK) {
            if (RxHeader.Identifier == 1) {
                return true;
            }
        }
    }
    if (HAL_FDCAN_GetRxFifoFillLevel(fdcan, FDCAN_RX_FIFO1) != 0) {
        FDCAN_RxHeaderTypeDef RxHeader;
        if (HAL_FDCAN_GetRxMessage(fdcan, FDCAN_RX_FIFO1, &RxHeader, data) == HAL_OK) {
            if (RxHeader.Identifier == 1) {
                return true;
            }
        }
    }

    return false;
}


#endif
