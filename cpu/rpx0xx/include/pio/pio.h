/*
 * Copyright (C) 2021 Otto-von-Guericke-Universität Magdeburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup         cpu_rpx0xx
 * @{
 *
 * @file
 * @brief           Internal PIO interface
 *
 * @author          Fabian Hüßler <fabian.huessler@ovgu.de>
*/

#ifndef PIO_PIO_H
#define PIO_PIO_H

#include "periph/pio.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Type to represent the width of an instruction
 */
typedef uint16_t pio_instr_t;

/**
 * @name    PIO instruction set
 * @{
 */
/**
 * @brief   JMP opcode
 */
#define PIO_INST_JMP                    (0u << 13)
/**
 * @brief   JMP opcode mask
 */
#define PIO_INST_JMP_MASK               (7u << 13)
/**
 * @brief   JMP condition shift position
 */
#define PIO_INST_JMP_CONDITION_SHIFT    5
/**
 * @brief   JMP condition mask
 */
#define PIO_INST_JMP_CONDITION_MASK     (7u << PIO_INST_JMP_CONDITION_SHIFT)
/**
 * @brief   JMP address mask
 */
#define PIO_INST_JMP_ADDRESS_MASK       (31u)
/**
 * @brief   JMP conditions
 */
typedef enum {
    PIO_INST_JMP_COND_NONE              = 0,    /**< Unconditionally */
    PIO_INST_JMP_COND_X_ZERO            = 1,    /**< If scratch X is 0 */
    PIO_INST_JMP_COND_X_DEC             = 2,    /**< If scratch X-- is not 0 */
    PIO_INST_JMP_COND_Y_ZERO            = 3,    /**< If scratch Y is 0 */
    PIO_INST_JMP_COND_Y_DEC             = 4,    /**< If scratch Y-- is not 0 */
    PIO_INST_JMP_COND_NOT_X_EQ_Y        = 5,    /**< If scratch X != scratch Y */
    PIO_INST_JMP_COND_PIN               = 6,    /**< If jump pin is high */
    PIO_INST_JMP_COND_NOT_OSR_EMPTY     = 7     /**< If OSR is not empty */
} pio_inst_jmp_cond_t;
/**
 * @brief   WAIT opcode
 */
#define PIO_INST_WAIT                   (1u << 13)
/**
 * @brief   WAIT opcode mask
 */
#define PIO_INST_WAIT_MASK              (7u << 13)
/**
 * @brief   WAIT polarity shift position
 */
#define PIO_INST_WAIT_POL_SHIFT         7
/**
 * @brief   WAIT polarity mask
 */
#define PIO_INST_WAIT_POL_MASK          (1u << PIO_INST_WAIT_POL_SHIFT)
/**
 * @brief   WAIT source shift positions
 */
#define PIO_INST_WAIT_SOURCE_SHIFT      5
/**
 * @brief   WAIT source mask
 */
#define PIO_INST_WAIT_SOURCE_MASK       (3u << PIO_INST_WAIT_SOURCE_SHIFT)
/**
 * @brief   WAIT index mask
 */
#define PIO_INST_WAIT_INDEX_MASK        (31u)
/**
 * @brief   WAIT polarities
 */
typedef enum {
    PIO_INST_WAIT_POL_LOW               = 0,    /**< Wait for 0 */
    PIO_INST_WAIT_POL_HIGH              = 1,    /**< Wait for 1 */
} pio_inst_wait_pol_t;
/**
 * @brief   WAIT sources
 */
typedef enum {
    PIO_INST_WAIT_SRC_GPIO              = 0,    /**< Wait on GPIO */
    PIO_INST_WAIT_SRC_PIN               = 1,    /**< Wait on input pin */
    PIO_INST_WAIT_SRC_IRQ               = 2     /**< Wait on interrupt */
} pio_inst_wait_src_t;
/**
 * @brief   IN opcode
 */
#define PIO_INST_IN                     (2u << 13)
/**
 * @brief   IN opcode mask
 */
#define PIO_INST_IN_MASK                (7u << 13)
/**
 * @brief   IN source shift position
 */
#define PIO_INST_IN_SOURCE_SHIFT        5
/**
 * @brief   IN source mask
 */
#define PIO_INST_IN_SOURCE_MASK         (7u << PIO_INST_IN_SOURCE_SHIFT)
/**
 * @brief   Number of bits to be shifted into the input shift register
 */
#define PIO_INST_IN_BIT_COUNT_MASK      (31u)
/**
 * @brief   IN sources
 */
typedef enum {
    PIO_INST_IN_SRC_PINS                = 0,    /**< From input pins */
    PIO_INST_IN_SRC_X                   = 1,    /**< From scratch X */
    PIO_INST_IN_SRC_Y                   = 2,    /**< From scratch Y */
    PIO_INST_IN_SRC_NULL                = 3,    /**< Fill with zeros */
    PIO_INST_IN_SRC_ISR                 = 6,    /**< From input shift register */
    PIO_INST_IN_SRC_OSR                 = 7     /**< From output shift register */
} pio_inst_in_src_t;
/**
 * @brief   OUT opcode
 */
#define PIO_INST_OUT                    (3u << 13)
/**
 * @brief   OUT opcode mask
 */
#define PIO_INST_OUT_MASK               (7u << 13)
/**
 * @brief   OUT destination shift position
 */
#define PIO_INST_OUT_DESTINATION_SHIFT  5
/**
 * @brief   OUT destination mask
 */
#define PIO_INST_OUT_DESTINATION_MASK   (7u << PIO_INST_OUT_DESTINATION_SHIFT)
/**
 * @brief   Number of bits to be shifted out of the output shift register
 */
#define PIO_INST_OUT_BIT_COUNT_MASK     (31u)
/**
 * @brief   OUT destinations
 */
typedef enum {
    PIO_INST_OUT_DST_PINS               = 0,    /**< To output pins */
    PIO_INST_OUT_DST_X                  = 1,    /**< To scratch X */
    PIO_INST_OUT_DST_Y                  = 2,    /**< To scratch Y */
    PIO_INST_OUT_DST_NULL               = 3,    /**< Discard data */
    PIO_INST_OUT_DST_PINDIRS            = 4,    /**< To output pin directions */
    PIO_INST_OUT_DST_PC                 = 5,    /**< To program counter */
    PIO_INST_OUT_DST_ISR                = 6,    /**< To input shift register */
    PIO_INST_OUT_DST_EXEC               = 7     /**< Execute as instruction */
} pio_inst_out_dst_t;
/**
 * @brief   PUSH opcode
 */
#define PIO_INST_PUSH                   (4u << 13)
/**
 * @brief   PUSH opcode mask
 */
#define PIO_INST_PUSH_MASK              ((7u << 13) | (1u << 7) | 31u)
/**
 * @brief   PUSH if input shift register is full shift position
 */
#define PIO_INST_PUSH_IF_FULL_SHIFT     6
/**
 * @brief   PUSH if input shift register is full mask
 */
#define PIO_INST_PUSH_IF_FULL_MASK      (1u << PIO_INST_PUSH_IF_FULL_SHIFT)
/**
 * @brief   PUSH if RX FIFO is not full shift position
 */
#define PIO_INST_PUSH_BLOCK_SHIFT       5
/**
 * @brief   PUSH if RX FIFO is not full mask
 */
#define PIO_INST_PUSH_BLOCK_MASK        (1u << PIO_INST_PUSH_BLOCK_SHIFT)
/**
 * @brief   PULL opcode
 */
#define PIO_INST_PULL                   ((4u << 13) | (1u << 7))
/**
 * @brief   PULL opcode mask
 */
#define PIO_INST_PULL_MASK              ((7u << 13) | (1u << 7) | 31u)
/**
 * @brief    PULL if output shift register is empty flag shift position
 */
#define PIO_INST_PULL_IF_EMPTY_SHIFT    6
/**
 * @brief    PULL if output shift register is empty mask
 */
#define PIO_INST_PULL_IF_EMPTY_MASK     (1u << PIO_INST_PULL_IF_EMPTY_SHIFT)
/**
 * @brief   PULL if TX FIFO is not empty flag shift position
 */
#define PIO_INST_PULL_BLOCK_SHIFT       5
/**
 * @brief   PULL if TX FIFO is not empty mask
 */
#define PIO_INST_PULL_BLOCK_MASK        (1u << PIO_INST_PULL_BLOCK_SHIFT)
/**
 * @brief   MOV opcode
 */
#define PIO_INST_MOV                    (5u << 13)
/**
 * @brief   MOV opcode mask
 */
#define PIO_INST_MOV_MASK               (7u << 13)
/**
 * @brief   MOV destination shift position
 */
#define PIO_INST_MOV_DESTINATION_SHIFT  5
/**
 * @brief   MOV destination mask
 */
#define PIO_INST_MOV_DESTINATION_MASK   (7u << PIO_INST_MOV_DESTINATION_SHIFT)
/**
 * @brief   MOV operation shift position
 */
#define PIO_INST_MOV_OP_SHIFT           3
/**
 * @brief   MOV operation mask
 */
#define PIO_INST_MOV_OP_MASK            (3u << PIO_INST_MOV_OP_SHIFT)
/**
 * @brief   MOV source shift position
 */
#define PIO_INST_MOV_SOURCE_SHIFT       0
/**
 * @brief   MOV source mask
 */
#define PIO_INST_MOV_SOURCE_MASK        (7u)
/**
 * @brief   MOV destinations
 */
typedef enum {
    PIO_INST_MOV_DST_PINS               = 0,    /**< To output pins */
    PIO_INST_MOV_DST_X                  = 1,    /**< To scratch X */
    PIO_INST_MOV_DST_Y                  = 2,    /**< To scratch Y */
    PIO_INST_MOV_DST_EXEC               = 4,    /**< Execute data as instruction */
    PIO_INST_MOV_DST_PC                 = 5,    /**< To program counter */
    PIO_INST_MOV_DST_ISR                = 6,    /**< To input shift register */
    PIO_INST_MOV_DST_OSR                = 7     /**< To output shift register */
} pio_inst_mov_dst_t;
/**
 * @brief MOV operation
 */
typedef enum {
    PIO_INST_MOV_OP_NONE                = 0,    /**< No operation */
    PIO_INST_MOV_OP_INVERT              = 1,    /**< Bitwise complement */
    PIO_INST_MOV_OP_REVERSE             = 2     /**< Bitwise reverse */
} pio_inst_mov_op_t;
/**
 * @brief   MOV source
 */
typedef enum {
    PIO_INST_MOV_SRC_PINS               = 0,    /**< From input pins */
    PIO_INST_MOV_SRC_X                  = 1,    /**< From scratch X */
    PIO_INST_MOV_SRC_Y                  = 2,    /**< From scratch Y */
    PIO_INST_MOV_SRC_NULL               = 3,    /**< Fill with zeros */
    PIO_INST_MOV_SRC_STATUS             = 5,    /**< From status */
    PIO_INST_MOV_SRC_ISR                = 6,    /**< From input shift register */
    PIO_INST_MOV_SRC_OSR                = 7     /**< From output shift register */
} pio_inst_mov_src_t;
/**
 * @brief   IRQ opcode
 */
#define PIO_INST_IRQ                    (6u << 13)
/**
 * @brief   IRQ opcode mask
 */
#define PIO_INST_IRQ_MASK               ((7u << 13) | (1u << 7))
/**
 * @brief   IRQ clear flag shift position
 */
#define PIO_INST_IRQ_CLR_SHIFT          6
/**
 * @brief   IRQ clear flag mask
 */
#define PIO_INST_IRQ_CLR_MASK           (1u << PIO_INST_IRQ_CLR_SHIFT)
/**
 * @brief   IRQ wait until clreared flag shift position
 */
#define PIO_INST_IRQ_WAIT_SHIFT         5
/**
 * @brief   IRQ wait until cleared flag mask
 */
#define PIO_INST_IRQ_WAIT_MASK          (1u << PIO_INST_IRQ_WAIT_SHIFT)
/**
 * @brief   IRQ index mask
 */
#define PIO_INST_IRQ_INDEX_MASK         (31u)
/**
 * @brief   SET opcode
 */
#define PIO_INST_SET                    (7u << 13)
/**
 * @brief   SET opcode mask
 */
#define PIO_INST_SET_MASK               (7u << 13)
/**
 * @brief   SET destination shift position
 */
#define PIO_INST_SET_DESTINATION_SHIFT  5
/**
 * @brief   SET destination mask
 */
#define PIO_INST_SET_DESTINATION_MASK   (7u << PIO_INST_SET_DESTINATION_SHIFT)
/**
 * @brief   SET data mask
 */
#define PIO_INST_SET_DATA_MASK          (31u)
/**
 * @brief   SET destinations
 */
typedef enum {
    PIO_INST_SET_DST_PINS               = 0,    /**< To set pins */
    PIO_INST_SET_DST_X                  = 1,    /**< To scratch X */
    PIO_INST_SET_DST_Y                  = 2,    /**< To scratch Y */
    PIO_INST_SET_DST_PINDIRS            = 4     /**< To set pin directions */
} pio_inst_set_dst_t;

/**
 * @brief   Construct a JMP instruction
 *
 * @param[in]       condition       Jump if this condition is true
 * @param[in]       address         Address in instruction memory to jump to
 *
 * @return  JMP instruction
 */
static inline pio_instr_t pio_inst_jmp(pio_inst_jmp_cond_t condition,
                                       unsigned address)
{
    return PIO_INST_JMP |
           (((unsigned)condition) << PIO_INST_JMP_CONDITION_SHIFT) |
           address;
}
/**
 * @brief   Construct a WAIT instruction
 *
 * @param[in]       polarity        Polarity
 * @param[in]       source          Which source to wait on
 * @param[in]       relative        IRQ index is relative to SM index
 * @param[in]       index           Which GPIO/PIN/IRQ to wait on
 *
 * @return  WAIT instruction
 */
static inline pio_instr_t pio_inst_wait(pio_inst_wait_pol_t polarity,
                                        pio_inst_wait_src_t source,
                                        bool relative,
                                        unsigned index)
{
    return PIO_INST_WAIT |
           ((unsigned)polarity << PIO_INST_WAIT_POL_SHIFT) |
           ((unsigned)source << PIO_INST_WAIT_SOURCE_SHIFT) |
           ((!!relative) << 4) | index;
}
/**
 * @brief   Construct an IN instruction
 *
 * @param[in]       source          From where to shift
 * @param[in]       bit_count       Number of bits to shift
 *
 * @return  IN instruction
 */
static inline pio_instr_t pio_inst_in(pio_inst_in_src_t source,
                                      unsigned bit_count)
{
    return PIO_INST_IN |
           ((unsigned)source << PIO_INST_IN_SOURCE_SHIFT) |
           (bit_count % 32);
}
/**
 * @brief   Construct an OUT instruction
 *
 * @param[in]       destination     To where to shift
 * @param[in]       bit_count       Number of bits to shift
 *
 * @return  OUT instruction
 */
static inline pio_instr_t pio_inst_out(pio_inst_out_dst_t destination,
                                       unsigned bit_count)
{
    return PIO_INST_OUT |
           ((unsigned)destination << PIO_INST_OUT_DESTINATION_SHIFT) |
           (bit_count % 32);
}
/**
 * @brief   Construct a PUSH instruction
 *
 * @param[in]       if_full         Only push if shift threshold is reached
 * @param[in]       block           Stall if RX FIFO is full
 *
 * @return  PUSH instruction
 */
static inline pio_instr_t pio_inst_push(bool if_full,
                                        bool block)
{
    return PIO_INST_PUSH |
           ((!!if_full) << PIO_INST_PUSH_IF_FULL_SHIFT) |
           ((!!block) << PIO_INST_PUSH_BLOCK_SHIFT);
}
/**
 * @brief   Construct a PULL instruction
 *
 * @param[in]       if_empty        Only pull if shift threshold is reached
 * @param[in]       block           Block if TX FIFO is empty
 *
 * @return  PULL instruction
 */
static inline pio_instr_t pio_inst_pull(bool if_empty,
                                        bool block)
{
    return PIO_INST_PULL |
           ((!!if_empty) << PIO_INST_PULL_IF_EMPTY_SHIFT) |
           ((!!block) << PIO_INST_PULL_BLOCK_SHIFT);
}
/**
 * @brief   Construct a MOV instruction
 *
 * @param[in]       destination     Where to move to
 * @param[in]       operation       Operation to perform on move
 * @param[in]       source          Where to move from
 *
 * @return  MOV instruction
 */
static inline pio_instr_t pio_inst_mov(pio_inst_mov_dst_t destination,
                                       pio_inst_mov_op_t operation,
                                       pio_inst_mov_src_t source)
{
    return PIO_INST_MOV |
           ((unsigned)destination << PIO_INST_MOV_DESTINATION_SHIFT) |
           ((unsigned)operation << PIO_INST_MOV_OP_SHIFT) |
           ((unsigned)source << PIO_INST_MOV_SOURCE_SHIFT);
}
/**
 * @brief   Construct an IRQ instruction
 *
 * @param[in]       clear           Clear IRQ instead to raise it
 * @param[in]       wait            Wait until IRQ is cleared if raised
 * @param[in]       relative        IRQ index is relative to SM number
 * @param[in]       index           IRQ index
 *
 * @return  IRQ instruction
 */
static inline pio_instr_t pio_inst_irq(bool clear,
                                       bool wait,
                                       bool relative,
                                       unsigned index)
{
    return PIO_INST_IRQ |
           (!!clear << PIO_INST_IRQ_CLR_SHIFT) |
           (!!wait << PIO_INST_IRQ_WAIT_SHIFT) |
           ((!!relative << 4) | index);
}
/**
 * @brief   Construct a SET instruction
 *
 * @param[in]       destination     Set destination
 * @param[in]       data            Data to set
 *
 * @return  SET instruction
 */
static inline pio_instr_t pio_inst_set(pio_inst_set_dst_t destination,
                                       unsigned data)
{
    return PIO_INST_SET |
           ((unsigned)destination << PIO_INST_SET_DESTINATION_SHIFT) |
           data;
}

/**
 * @brief   Encode the delay/sideset instruction field
 *
 * @param[in]       sideset         Sideset value to write to sideset pins
 * @param[in]       sideset_count   Number of sideset pins
 * @param[in]       sideset_opt     If sideset is optional
 * @param[in]       delay           Delay cycles
 *
 * @return  Delay/sideset encoding that can be added to an instruction
 */
static inline pio_instr_t pio_inst_delay_sideset(unsigned sideset,
                                                 unsigned sideset_count,
                                                 bool sideset_opt,
                                                 unsigned delay)
{
    return ((!!sideset_opt) << 12) |
           (sideset << (13 - (!!sideset_opt + sideset_count))) |
           (delay << 8);
}
/** @} */

/**
 * @brief   Internal state machine registers
 */
typedef struct pio_sm_ctrl_regs {
    volatile uint32_t clkdiv;       /**< SMx_CLKDIV */
    volatile uint32_t execctrl;     /**< SMx_EXECCTRL*/
    volatile uint32_t shiftctrl;    /**< SMx_SHIFTCTRL */
    const volatile uint32_t addr;   /**< SMx_ADDR */
    volatile uint32_t instr;        /**< SMx_INSTR */
    volatile uint32_t pinctrl;      /**< SMx_PINCTRL */
} pio_sm_ctrl_regs_t;

/**
 * @brief   State machine configuration registers
 */
#define PIO_SM_CTRL_BASE(dev)   ((pio_sm_ctrl_regs_t *)(&((dev)->SM0_CLKDIV)))

/**
 * @brief   Maximum clock divider
 */
#define PIO_SM_CLKDIV_MAX               65536

/**
 * @brief   Convert IRQ index to bitmask
 */
#define PIO_IRQ_MASK(irq)               (1u << (irq))

/**
 * @brief   Convert state machine index to bitmask
 */
#define PIO_SM_MASK(sm)                 (1u << (sm))

/**
 * @brief   PIO IRQ flags
 */
typedef enum {
    PIO_SM_IRQ0_SM                      = PIO_IRQ_MASK(0),          /**< SM IRQ0 */
    PIO_SM_IRQ0_TXNFULL                 = PIO_IRQ_MASK(1),          /**< TX FIFO not full IRQ0 */
    PIO_SM_IRQ0_RXNEMPTY                = PIO_IRQ_MASK(2),          /**< RX FIFO not empty IRQ0 */
    PIO_SM_IRQ1_SM                      = PIO_IRQ_MASK(3),          /**< SM IRQ1 */
    PIO_SM_IRQ1_TXNFULL                 = PIO_IRQ_MASK(4),          /**< TX FIFO not full IRQ1 */
    PIO_SM_IRQ1_RXNEMPTY                = PIO_IRQ_MASK(5),          /**< RX FIFO not empty IRQ1 */
    PIO_SM_IRQ_ALL                      = (PIO_SM_IRQ0_SM       |
                                           PIO_SM_IRQ0_TXNFULL  |
                                           PIO_SM_IRQ0_RXNEMPTY |
                                           PIO_SM_IRQ1_SM       |
                                           PIO_SM_IRQ1_TXNFULL  |
                                           PIO_SM_IRQ1_RXNEMPTY)    /**< All flags above */
} pio_sm_irq_mask_t;

/**
 * @brief   PIO state machine flags
 */
typedef enum {
    PIO_SM0                             = PIO_SM_MASK(0),           /**< Mask bit of SM 0 */
    PIO_SM1                             = PIO_SM_MASK(1),           /**< Mask bit of SM 1 */
    PIO_SM2                             = PIO_SM_MASK(2),           /**< Mask bit of SM 2 */
    PIO_SM3                             = PIO_SM_MASK(3),           /**< Mask bit of SM 3 */
    PIO_SM_ALL                          = (PIO_SM0 | PIO_SM1 |
                                           PIO_SM2 | PIO_SM3)       /**< All flags above */
} pio_sm_mask_t;

/**
 * @brief   PIO state machine interrupt callbacks
 */
typedef struct pio_isr_vec {
    void (*sm)(pio_t pio, pio_sm_t sm);         /**< Called when SM issues an interrupt */
    void (*tx_ready)(pio_t pio, pio_sm_t sm);   /**< Called when Tx FIFO is not full */
    void (*rx_ready)(pio_t pio, pio_sm_t sm);   /**< Called when Rx FIFO is not empty */
} pio_isr_vec_t;

/**
 * @brief   PIO program configuration
 */
typedef struct pio_program_conf {
    unsigned pc_start;          /**< Initial program counter */
    unsigned wrap_bottom;       /**< Instruction index after which the PC wraps around */
    unsigned wrap_top;          /**< Instruction index the PC wraps to */
    unsigned sideset_count;     /**< Number of bits used for sideset */
    bool sideset_optional;      /**< Whether the sideset is optional */
    bool sideset_pindirs;       /**< Whether the sideset effects pin directions */
} pio_program_conf_t;

/**
 * @brief   PIO clock configuration
 */
typedef struct pio_sm_clkdiv {
    uint16_t div;       /**< Integer divider */
    uint8_t frac_100;   /**< Fractional divider, two digits after comma */
} pio_sm_clkdiv_t;

/**
 * @brief   Execute a single instruction
 *
 * This function does not wait until the instruction has completed.
 *
 * @param[in]       pio         PIO index
 * @param[in]       sm          PIO state machine index
 * @param[in]       inst        Instruction to be executed
 *
 * @return  Success: 0
 *          Failure: < 0 (state machine stalls on a previous instruction)
 */
int pio_sm_exec(pio_t pio, pio_sm_t sm, pio_instr_t inst);

/**
 * @brief   Execute a single instruction
 *
 * This function blocks until the instruction has completed.
 *
 * @param[in]       pio         PIO index
 * @param[in]       sm          PIO state machine index
 * @param[in]       inst        Instruction to be executed
 */
void pio_sm_exec_block(pio_t pio, pio_sm_t sm, pio_instr_t inst);

/**
 * @brief   Write program instructions to their allocated location
 *
 * @param[in]       pio         PIO index
 * @param[in]       prog        PIO program to write
 * @param[in]       instr       Program instructions
 *
 * @return  Success: 0
 *          Failure: != 0
 */
int pio_write_program(pio_t pio, const pio_program_t *prog, const pio_instr_t *instr);

/**
 * @brief   Apply the default state machine configuration
 *
 * @param[in]       pio         PIO index
 * @param[in]       sm          PIO state machine index
 */
void pio_sm_reset(pio_t pio, pio_sm_t sm);

/**
 * @brief   Restart a state machine
 *
 * @param[in]       pio         PIO index
 * @param[in]       sm          PIO state machine index
 */
void pio_sm_restart(pio_t pio, pio_sm_t sm);

/**
 * @brief   Set ISR callbacks for a state machine
 *
 * @param[in]       pio         PIO index
 * @param[in]       sm          PIO state machine index
 * @param[in]       vec         State machine ISR callback vector
 */
void pio_sm_set_isr_vec(pio_t pio, pio_sm_t sm, const pio_isr_vec_t *vec);

/**
 * @brief   Configure which pins are writeable by an 'out pins' instruction
 *
 * The pins start at @p pin_base and are @p pin_count many.
 * The pin sequence wraps around after the maximum pin.
 *
 * @param[in]       pio         PIO index
 * @param[in]       sm          PIO state machine index
 * @param[in]       pin_base    First pin of a pin sequence effected by an out instruction
 * @param[in]       pin_count   Number of out pins
 */
void pio_sm_set_out_pins(pio_t pio, pio_sm_t sm, gpio_t pin_base, unsigned pin_count);

/**
 * @brief   Configure the state machine input pin mapping
 *
 * The pin sequence wraps around after the maximum pin.
 *
 * @param[in]       pio         PIO index
 * @param[in]       sm          PIO state machine index
 * @param[in]       pin_base    First pin of a pin sequence effected by an in instruction
 */
void pio_sm_set_in_pins(pio_t pio, pio_sm_t sm, gpio_t pin_base);

/**
 * @brief   Configure which pins are effected by a 'set pins' instructions.
 *
 * The pins start at @p pin_base and are @p pin_count many.
 * The pin sequence wraps around after the maximum pin.
 *
 * @param[in]       pio         PIO index
 * @param[in]       sm          PIO state machine index
 * @param[in]       pin_base    First pin of a pin sequence effected by a set instruction
 * @param[in]       pin_count   Number of set pins
 */
void pio_sm_set_set_pins(pio_t pio, pio_sm_t sm, gpio_t pin_base, unsigned pin_count);

/**
 * @brief   Set the first pin of a sequence of sideset count pins which
 *          are effected by sideset instructions
 *
 * @param[in]       pio         PIO index
 * @param[in]       sm          PIO state machine index
 * @param[in]       pin_base    First pin of a pin sequence effected by a sideset instruction
 */
void pio_sm_set_sideset_pins(pio_t pio, pio_sm_t sm, gpio_t pin_base);

/**
 * @brief   Configure how many pins are sideset pins and whether a sideset
 *          is optional or required for every instruction
 *
 * Each sideset pin occupies one bit of the delay field of an instruction.
 * If sidesets are optional, one more bit is used.
 *
 * @param[in]       pio         PIO index
 * @param[in]       sm          PIO state machine index
 * @param[in]       pin_count   Number of pins effected by a sideset instruction
 * @param[in]       enable      Whether the sideset is optional
 */
void pio_sm_set_sideset_count(pio_t pio, pio_sm_t sm, unsigned pin_count, bool enable);

/**
 * @brief   Configure whether a sideset effects pins or pin dircetions
 *
 * @param[in]       pio         PIO index
 * @param[in]       sm          PIO state machine index
 * @param[in]       pindir      Whether the sideset effects pin directions
 */
void pio_sm_set_sideset_target(pio_t pio, pio_sm_t sm, bool pindir);

/**
 * @brief   Create a clock divider struct
 *
 * @warning If the resulting clock divisor @ref CLOCK_CORECLOCK / @p f_hz
 *          would be greater than @ref PIO_SM_CLKDIV_MAX, an assert is fired.
 *
 * @param[in]       f_hz        Desired PIO frequency
 *
 * @return  Clock divider struct
 */
pio_sm_clkdiv_t pio_sm_clkdiv(uint32_t f_hz);

/**
 * @brief   Apply the clock configuration to a PIO
 *
 * There is an integer and a fractional portion of the clock divider.
 * The fractional parameter is in 1/100.
 * That means 90 means .90 and 9 means .09.
 * The PIOs are running from the system cLock.
 *
 * @param[in]       pio         PIO index
 * @param[in]       sm          PIO state machine index
 * @param[in]       clk         Clock configuration
 */
void pio_sm_set_clkdiv(pio_t pio, pio_sm_t sm, pio_sm_clkdiv_t clk);

/**
 * @brief   Restart the clock divider of several state machines.
 *
 * This is useful if several state machines must be synchronized
 * and to resync after the clock divider has been changed on-the-fly.
 *
 * @param[in]       pio         PIO index
 * @param[in]       sm_mask     PIO state machine mask @ref pio_sm_mask_t
 */
void pio_sm_clkdiv_restart(pio_t pio, unsigned sm_mask);

/**
 * @brief   Set program instruction wrap boundaries.
 *
 * @param[in]       pio         PIO index
 * @param[in]       sm          PIO state machine index
 * @param[in]       prog_loc    Program location
 * @param[in]       top         wrap from
 * @param[in]       bottom      wrap to
 */
void pio_sm_set_wrap(pio_t pio, pio_sm_t sm, unsigned prog_loc, uint8_t top, uint8_t bottom);

/**
 * @brief   Configure the pin on which to branch on a 'jmp pins' instruction.
 *
 * The jump is executed if the pin is high.
 *
 * @param[in]       pio         PIO index
 * @param[in]       sm          PIO state machine index
 * @param[in]       pin         Branch pin
 */
void pio_sm_set_jmp_pin(pio_t pio, pio_sm_t sm, gpio_t pin);

/**
 * @brief   Configure the shift in behaviour of a state machine
 *
 * @param[in]       pio         PIO index
 * @param[in]       sm          PIO state machine index
 * @param[in]       right       Whether to shift right instead of left
 * @param[in]       autopush    Whether to automatically push on reached threshold
 * @param[in]       threshold   In shift threshold
 */
void pio_sm_set_in_shift(pio_t pio, pio_sm_t sm, bool right, bool autopush, unsigned threshold);

/**
 * @brief   Configure the shift out behaviour of a state machine
 *
 * @param[in]       pio         PIO index
 * @param[in]       sm          PIO state machine index
 * @param[in]       right       Whether to shift right instead of left
 * @param[in]       autopull    Whether to automatically pull on reached threshold
 * @param[in]       threshold   Out shift threshould
 */
void pio_sm_set_out_shift(pio_t pio, pio_sm_t sm, bool right, bool autopull, unsigned threshold);

/**
 * @brief   Join the TX FIFO to the RX FIFO
 *
 * @param[in]       pio         PIO index
 * @param[in]       sm          PIO state machine index
 */
void pio_sm_set_fifo_join_rx(pio_t pio, pio_sm_t sm);

/**
 * @brief   Join the RX FIFO to the TX FIFO
 *
 * @param[in]       pio         PIO index
 * @param[in]       sm          PIO state machine index
 */
void pio_sm_set_fifo_join_tx(pio_t pio, pio_sm_t sm);

/**
 * @brief   No joinded FIFO
 *
 * @param[in]       pio         PIO index
 * @param[in]       sm          PIO state machine index
 */
void pio_sm_reset_fifos(pio_t pio, pio_sm_t sm);

/**
 * @brief   Drop all pending words in the tx and rx FIFO
 *
 * @param[in]       pio         PIO index
 * @param[in]       sm          PIO state machine index
 */
void pio_sm_clear_fifos(pio_t pio, pio_sm_t sm);

/**
 * @brief   Enable PIO interrupts
 *
 * @param[in]       pio         PIO index
 * @param[in]       sm          PIO state machine index
 * @param[in]       irq_mask    IRQ flags to enable @ref pio_sm_irq_mask_t
 */
void pio_sm_irq_enable(pio_t pio, pio_sm_t sm, unsigned irq_mask);

/**
 * @brief   Disable PIO interrupts
 *
 * @param[in]       pio         PIO index
 * @param[in]       sm          PIO state machine index
 * @param[in]       irq_mask    IRQ flags to disable @ref pio_sm_irq_mask_t
 */
void pio_sm_irq_disable(pio_t pio, pio_sm_t sm, unsigned irq_mask);

/**
 * @brief   Send one word to a state machine
 *
 * This function does not block but returns a negative integer if the
 * FIFO is fill.
 *
 * @param[in]       pio         PIO index
 * @param[in]       sm          PIO state machine index
 * @param[in]       word        Word to transmit
 *
 * @return  Success: 0
 *          Failure: != 0
 */
int pio_sm_transmit_word(pio_t pio, pio_sm_t sm, uint32_t word);

/**
 * @brief   Send one word to a state machine
 *
 * This function blocks until one word has been sent.
 *
 * @param[in]       pio         PIO index
 * @param[in]       sm          PIO state machine index
 * @param[in]       word        Word to transmit
 */
void pio_sm_transmit_word_block(pio_t pio, pio_sm_t sm, uint32_t word);

/**
 * @brief   Send @p count words to a state machine
 *
 * This function blocks until @p count words have been sent.
 *
 * @param[in]       pio         PIO index
 * @param[in]       sm          PIO state machine index
 * @param[in]       words       Words to transmit
 * @param[in]       count       Number of words
 */
void pio_sm_transmit_words_block(pio_t pio, pio_sm_t sm, const uint32_t *words, unsigned count);

/**
 * @brief   Receive a word from a state machine
 *
 * This function does not block but returns a negative integer if there is
 * no data.
 *
 * @param[in]       pio         PIO index
 * @param[in]       sm          PIO state machine index
 * @param[out]      word        Destination to store the received word
 *
 * @return  Success: 0
 *          Failure: != 0
 */
int pio_sm_receive_word(pio_t pio, pio_sm_t sm, uint32_t *word);

/**
 * @brief   Receive one word from a state machine
 *
 * This function blocks until one word has been received.
 *
 * @param[in]       pio         PIO index
 * @param[in]       sm          PIO state machine index
 * @param[out]      word        Destination to store the received word
 */
void pio_sm_receive_word_block(pio_t pio, pio_sm_t sm, uint32_t *word);

/**
 * @brief   Receive words from a state machine.
 *
 * This function blocks until @p count words have been received.
 *
 * @param[in]       pio         PIO index
 * @param[in]       sm          PIO state machine index
 * @param[out]      words       Buffer to store received words
 * @param[in]       count       Number of words
 */
void pio_sm_receive_words_block(pio_t pio, pio_sm_t sm, uint32_t *words, unsigned count);

/**
 * @brief   Read interrupt flags
 *
 * @param[in]       pio         PIO index
 *
 * @return  IRQ flags
 */
uint32_t pio_irq_get(pio_t pio);

/**
 * @brief   Clear interrupt flags
 *
 * @param[in]       pio         PIO index
 * @param[in]       irq_flags   Interrupt flags to clear
 */
void pio_irq_clear(pio_t pio, unsigned irq_flags);

/**
 * @brief   Apply common program configuration.
 *
 * This function should be called before any specific program
 * parameters are set because the state machine is reset first.
 *
 * @param[in]       pio         PIO index
 * @param[in]       sm          PIO state machine index
 * @param[in]       prog        Program
 * @param[in]       conf        Program configuration
 *
 * @return  Success: 0
 *          Failure: != 0
 */
int pio_sm_init_common(pio_t pio, pio_sm_t sm, const pio_program_t *prog,
                       const pio_program_conf_t *conf);

/**
 * @brief   Apply pin directions in @p values for which the corresponding bit in @p mask is set.
 *
 * 1 means output, 0 means input.
 *
 * @param[in]       pio         PIO index
 * @param[in]       sm          PIO state machine index
 * @param[in]       values      Pin directions out/in to be set
 * @param[in]       mask        Mask of which pins should be modified
 */
void pio_sm_set_pindirs_with_mask(pio_t pio, pio_sm_t sm, gpio_t values, gpio_t mask);

/**
 * @brief   Apply pin values in @p values for which the corresponding bit in @p mask is set.
 *
 * 1 means high, 0 means low.
 *
 * @param[in]       pio         PIO index
 * @param[in]       sm          PIO state machine index
 * @param[in]       values      Pin values on/off to be set
 * @param[in]       mask        Mask of which pins should be modified
 */
void pio_sm_set_pins_with_mask(pio_t pio, pio_sm_t sm, gpio_t values, gpio_t mask);

/**
 * @brief   Print status information about the FIFOs and programs
 *
 * @param[in]       pio         PIO index
 */
void pio_print_status(pio_t pio);

/**
 * @brief   Print debug information about the current state of this PIO
 *
 * @param[in]       pio         PIO index
 */
void pio_print_debug(pio_t pio);

/**
 * @brief   Clear TX stall debug flag
 *
 * @param[in]       pio         PIO index
 * @param[in]       sm_mask     SM mask
 */
static inline void pio_sm_clear_debug_txstall(pio_t pio, unsigned sm_mask)
{
    io_reg_atomic_set(&pio_config[pio].dev->FDEBUG.reg,
                      ((sm_mask & PIO_SM_ALL) << PIO0_FDEBUG_TXSTALL_Pos));
}

/**
 * @brief   Clear TX overflow debug flag
 *
 * @param[in]       pio         PIO index
 * @param[in]       sm_mask     SM mask
 */
static inline void pio_sm_clear_debug_txover(pio_t pio, unsigned sm_mask)
{
    io_reg_atomic_set(&pio_config[pio].dev->FDEBUG.reg,
                      ((sm_mask & PIO_SM_ALL) << PIO0_FDEBUG_TXOVER_Pos));
}

/**
 * @brief   Clear RX underflow debug flag
 *
 * @param[in]       pio         PIO index
 * @param[in]       sm_mask     SM mask
 */
static inline void pio_sm_clear_debug_rxunder(pio_t pio, unsigned sm_mask)
{
    io_reg_atomic_set(&pio_config[pio].dev->FDEBUG.reg,
                      ((sm_mask & PIO_SM_ALL) << PIO0_FDEBUG_RXUNDER_Pos));
}

/**
 * @brief   Clear RX stall debug flag
 *
 * @param[in]       pio         PIO index
 * @param[in]       sm_mask     SM mask
 */
static inline void pio_sm_clear_debug_rxstall(pio_t pio, unsigned sm_mask)
{
    io_reg_atomic_set(&pio_config[pio].dev->FDEBUG.reg,
                      ((sm_mask & PIO_SM_ALL) << PIO0_FDEBUG_RXSTALL_Pos));
}

/**
 * @brief   Check if TX FIFO is empty
 *
 * @param[in]       pio         PIO index
 * @param[in]       sm          PIO state machine index
 *
 * @return  If TX FIFO is empty
 */
static inline bool pio_sm_tx_fifo_empty(pio_t pio, pio_sm_t sm)
{
    return !!(pio_config[pio].dev->FSTAT.reg & ((1u << sm) << PIO0_FSTAT_TXEMPTY_Pos));
}

/**
 * @brief   Check if TX FIFO is full
 *
 * @param[in]       pio         PIO index
 * @param[in]       sm          PIO state machine index
 *
 * @return  If TX FIFO is full
 */
static inline bool pio_sm_tx_fifo_full(pio_t pio, pio_sm_t sm)
{
    return !!(pio_config[pio].dev->FSTAT.reg & ((1u << sm) << PIO0_FSTAT_TXFULL_Pos));
}

/**
 * @brief   Check if RX FIFO is empty
 *
 * @param[in]       pio         PIO index
 * @param[in]       sm          PIO state machine index
 *
 * @return  If RX FIFO is empty
 */
static inline bool pio_sm_rx_fifo_empty(pio_t pio, pio_sm_t sm)
{
    return !!(pio_config[pio].dev->FSTAT.reg & ((1u << sm) << PIO0_FSTAT_RXEMPTY_Pos));
}

/**
 * @brief   Check if RX FIFO is full
 *
 * @param[in]       pio         PIO index
 * @param[in]       sm          PIO state machine index
 *
 * @return  If RX FIFO is full
 */
static inline bool pio_sm_rx_fifo_full(pio_t pio, pio_sm_t sm)
{
    return !!((pio_config[pio].dev)->FSTAT.reg & ((1u << sm) << PIO0_FSTAT_RXFULL_Pos));
}

#ifdef __cplusplus
}
#endif

#endif /* PIO_PIO_H */
/** @} */
