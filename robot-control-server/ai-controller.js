const OpenAI = require('openai');

const SYSTEM_PROMPT = `You are an AI assistant controlling a mechanum wheel robot.
The robot has: 4 mechanum wheels (omnidirectional movement), a camera, a VL53L0X distance sensor, and an OLED display.

Available actions:
- Move in any direction (forward, backward, turn left/right, strafe left/right)
- Take a photo and recognize objects using AI vision
- Display text on the OLED screen (8 rows, 16 chars each)
- Toggle autonomous mode (auto-detect objects with distance sensor)

Respond naturally in the same language the user uses (Korean or English).
When the user asks you to move, take photos, or control the robot, call the appropriate tools.
For movement commands, use speed 150-255 (default 200). You can specify duration_ms for timed movements.
Keep responses concise and friendly.`;

const TOOLS = [
  {
    type: 'function',
    function: {
      name: 'move_forward',
      description: 'Move the robot forward',
      parameters: {
        type: 'object',
        properties: {
          speed: { type: 'integer', description: 'Motor speed 0-255', default: 200 },
          duration_ms: { type: 'integer', description: 'Duration in ms (0 = until stop command)', default: 0 }
        }
      }
    }
  },
  {
    type: 'function',
    function: {
      name: 'move_backward',
      description: 'Move the robot backward',
      parameters: {
        type: 'object',
        properties: {
          speed: { type: 'integer', description: 'Motor speed 0-255', default: 200 },
          duration_ms: { type: 'integer', description: 'Duration in ms (0 = until stop command)', default: 0 }
        }
      }
    }
  },
  {
    type: 'function',
    function: {
      name: 'stop_motors',
      description: 'Stop all motors immediately',
      parameters: { type: 'object', properties: {} }
    }
  },
  {
    type: 'function',
    function: {
      name: 'turn_left',
      description: 'Turn the robot left (rotate in place)',
      parameters: {
        type: 'object',
        properties: {
          speed: { type: 'integer', description: 'Motor speed 0-255', default: 200 },
          duration_ms: { type: 'integer', description: 'Duration in ms', default: 0 }
        }
      }
    }
  },
  {
    type: 'function',
    function: {
      name: 'turn_right',
      description: 'Turn the robot right (rotate in place)',
      parameters: {
        type: 'object',
        properties: {
          speed: { type: 'integer', description: 'Motor speed 0-255', default: 200 },
          duration_ms: { type: 'integer', description: 'Duration in ms', default: 0 }
        }
      }
    }
  },
  {
    type: 'function',
    function: {
      name: 'strafe_left',
      description: 'Strafe (slide) the robot to the left without turning',
      parameters: {
        type: 'object',
        properties: {
          speed: { type: 'integer', description: 'Motor speed 0-255', default: 200 },
          duration_ms: { type: 'integer', description: 'Duration in ms', default: 0 }
        }
      }
    }
  },
  {
    type: 'function',
    function: {
      name: 'strafe_right',
      description: 'Strafe (slide) the robot to the right without turning',
      parameters: {
        type: 'object',
        properties: {
          speed: { type: 'integer', description: 'Motor speed 0-255', default: 200 },
          duration_ms: { type: 'integer', description: 'Duration in ms', default: 0 }
        }
      }
    }
  },
  {
    type: 'function',
    function: {
      name: 'take_photo_and_recognize',
      description: 'Take a photo with the camera and recognize the object using AI vision. Returns the recognized object name. Note: this temporarily disconnects WiFi.',
      parameters: { type: 'object', properties: {} }
    }
  },
  {
    type: 'function',
    function: {
      name: 'display_text',
      description: 'Display text on the OLED screen',
      parameters: {
        type: 'object',
        properties: {
          row: { type: 'integer', description: 'Row number 0-7' },
          text: { type: 'string', description: 'Text to display (max 16 characters)' }
        },
        required: ['row', 'text']
      }
    }
  },
  {
    type: 'function',
    function: {
      name: 'set_autonomous_mode',
      description: 'Enable or disable autonomous object detection mode. When enabled, the robot automatically detects and recognizes objects using the distance sensor.',
      parameters: {
        type: 'object',
        properties: {
          enabled: { type: 'boolean', description: 'true to enable, false to disable' }
        },
        required: ['enabled']
      }
    }
  }
];

class AIController {
  constructor(sendCommand) {
    this.sendCommand = sendCommand;
    this.openai = new OpenAI({ apiKey: process.env.OPENAI_API_KEY });
    this.conversationHistory = [];
    this.maxHistory = 10;
  }

  async processMessage(userText) {
    this.conversationHistory.push({ role: 'user', content: userText });

    // Trim history
    if (this.conversationHistory.length > this.maxHistory * 2) {
      this.conversationHistory = this.conversationHistory.slice(-this.maxHistory * 2);
    }

    const messages = [
      { role: 'system', content: SYSTEM_PROMPT },
      ...this.conversationHistory
    ];

    const actions = [];
    let response;

    try {
      response = await this.openai.chat.completions.create({
        model: 'gpt-4o-mini',
        messages,
        tools: TOOLS,
        tool_choice: 'auto'
      });
    } catch (err) {
      throw new Error(`OpenAI API error: ${err.message}`);
    }

    let message = response.choices[0].message;

    // Process tool calls iteratively
    while (message.tool_calls && message.tool_calls.length > 0) {
      this.conversationHistory.push(message);

      for (const toolCall of message.tool_calls) {
        const fnName = toolCall.function.name;
        let fnArgs = {};
        try {
          fnArgs = JSON.parse(toolCall.function.arguments || '{}');
        } catch (e) { /* empty args */ }

        console.log(`[AI tool] ${fnName}`, fnArgs);
        actions.push({ tool: fnName, args: fnArgs });

        let result;
        try {
          const cmdResult = await this.sendCommand(fnName, fnArgs);
          result = JSON.stringify(cmdResult.data || { status: cmdResult.status });
        } catch (err) {
          result = JSON.stringify({ error: err.message });
        }

        this.conversationHistory.push({
          role: 'tool',
          tool_call_id: toolCall.id,
          content: result
        });
      }

      // Get next response
      try {
        response = await this.openai.chat.completions.create({
          model: 'gpt-4o-mini',
          messages: [
            { role: 'system', content: SYSTEM_PROMPT },
            ...this.conversationHistory
          ],
          tools: TOOLS,
          tool_choice: 'auto'
        });
      } catch (err) {
        throw new Error(`OpenAI API error: ${err.message}`);
      }

      message = response.choices[0].message;
    }

    const reply = message.content || '(no response)';
    this.conversationHistory.push({ role: 'assistant', content: reply });

    return { reply, actions };
  }
}

module.exports = { AIController };
