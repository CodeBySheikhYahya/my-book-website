/**
 * API Client - Generated from OpenAPI spec
 * 
 * To regenerate from spec, run: npm run generate-api
 * This file can be replaced by the generated client once you run the generation.
 */

import axios, { AxiosInstance } from 'axios';

// Types matching the OpenAPI spec
export interface ChatRequest {
  question: string;
  selected_text?: string | null;
  session_id?: string | null;
}

export interface Source {
  title: string;
  source: string;
  text: string;
}

export interface ChatResponse {
  answer: string;
  sources?: Source[] | null;
  session_id?: string | null;
}

export interface ChatHistoryItem {
  id?: number;
  session_id?: string;
  question?: string;
  answer?: string;
  selected_text?: string | null;
  sources?: Source[] | null;
  created_at?: string;
}

export interface IndexResponse {
  message: string;
}

export interface HealthResponse {
  status: string;
  service: string;
}

export interface HistoryResponse {
  session_id: string;
  history: ChatHistoryItem[];
}

/**
 * API Client class matching the OpenAPI spec
 */
export class ApiClient {
  private client: AxiosInstance;

  constructor(baseURL: string = 'http://localhost:8000') {
    this.client = axios.create({
      baseURL,
      headers: {
        'Content-Type': 'application/json',
      },
    });
  }

  /**
   * Chat endpoint - answers questions about the book
   */
  async chat(request: ChatRequest): Promise<ChatResponse> {
    const response = await this.client.post<ChatResponse>('/chat', request);
    return response.data;
  }

  /**
   * Index all book content into Qdrant
   */
  async indexBooks(): Promise<IndexResponse> {
    const response = await this.client.post<IndexResponse>('/index');
    return response.data;
  }

  /**
   * Get chat history for a session
   */
  async getHistory(sessionId: string, limit: number = 10): Promise<HistoryResponse> {
    const response = await this.client.get<HistoryResponse>(
      `/chat/history/${sessionId}`,
      { params: { limit } }
    );
    return response.data;
  }

  /**
   * Health check
   */
  async health(): Promise<HealthResponse> {
    const response = await this.client.get<HealthResponse>('/health');
    return response.data;
  }
}

// Export a default instance
export const apiClient = new ApiClient();

