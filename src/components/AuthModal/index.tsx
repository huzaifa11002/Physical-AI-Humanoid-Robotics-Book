import type {ReactNode} from 'react';
import {useState} from 'react';
import { authClient } from '../../lib/auth-client'; // Import the auth client
import styles from './styles.module.css';

interface AuthModalProps {
  isOpen: boolean;
  onClose: () => void;
  onSuccess?: () => void;
}

export default function AuthModal({isOpen, onClose, onSuccess}: AuthModalProps): ReactNode {
  const [isLogin, setIsLogin] = useState(true);
  const [formData, setFormData] = useState({
    name: '',
    email: '',
    password: '',
  });
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');

  if (!isOpen) return null;

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setLoading(true);
    setError('');

    try {
      const { email, password, name } = formData;
      
      let data, error;
      
      if (isLogin) {
        const res = await authClient.signIn.email({
             email,
             password,
        });
        
        data = res.data;
        error = res.error;
      } else {
         const res = await authClient.signUp.email({
            email,
            password,
            name,
        });
        
        data = res.data;
        error = res.error;
      }

      if (error) {
         throw new Error(error.message || 'Authentication failed');
      }

      onSuccess?.();
      onClose();
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred');
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className={styles.overlay} onClick={onClose}>
      <div className={styles.modal} onClick={(e) => e.stopPropagation()}>
        <button className={styles.closeButton} onClick={onClose}>
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none">
            <path d="M18 6L6 18M6 6l12 12" stroke="currentColor" strokeWidth="2" strokeLinecap="round"/>
          </svg>
        </button>

        <div className={styles.header}>
          <h2 className={styles.title}>{isLogin ? 'Welcome Back' : 'Create Account'}</h2>
          <p className={styles.subtitle}>
            {isLogin ? 'Sign in to continue your learning' : 'Join us to start learning'}
          </p>
        </div>

        <div className={styles.tabs}>
          <button
            className={`${styles.tab} ${isLogin ? styles.tabActive : ''}`}
            onClick={() => setIsLogin(true)}
          >
            Login
          </button>
          <button
            className={`${styles.tab} ${!isLogin ? styles.tabActive : ''}`}
            onClick={() => setIsLogin(false)}
          >
            Register
          </button>
        </div>

        <form className={styles.form} onSubmit={handleSubmit}>
          {!isLogin && (
            <div className={styles.inputGroup}>
              <label className={styles.label}>
                <svg className={styles.inputIcon} width="20" height="20" viewBox="0 0 24 24" fill="none">
                  <path d="M20 21v-2a4 4 0 0 0-4-4H8a4 4 0 0 0-4 4v2M12 11a4 4 0 1 0 0-8 4 4 0 0 0 0 8z" stroke="currentColor" strokeWidth="2" strokeLinecap="round"/>
                </svg>
                Name
              </label>
              <input
                type="text"
                className={styles.input}
                placeholder="Enter your name"
                value={formData.name}
                onChange={(e) => setFormData({...formData, name: e.target.value})}
                required={!isLogin}
              />
            </div>
          )}

          <div className={styles.inputGroup}>
            <label className={styles.label}>
              <svg className={styles.inputIcon} width="20" height="20" viewBox="0 0 24 24" fill="none">
                <path d="M4 4h16c1.1 0 2 .9 2 2v12c0 1.1-.9 2-2 2H4c-1.1 0-2-.9-2-2V6c0-1.1.9-2 2-2z" stroke="currentColor" strokeWidth="2"/>
                <path d="m22 6-10 7L2 6" stroke="currentColor" strokeWidth="2"/>
              </svg>
              Email
            </label>
            <input
              type="email"
              className={styles.input}
              placeholder="Enter your email"
              value={formData.email}
              onChange={(e) => setFormData({...formData, email: e.target.value})}
              required
            />
          </div>

          <div className={styles.inputGroup}>
            <label className={styles.label}>
              <svg className={styles.inputIcon} width="20" height="20" viewBox="0 0 24 24" fill="none">
                <rect x="3" y="11" width="18" height="11" rx="2" stroke="currentColor" strokeWidth="2"/>
                <path d="M7 11V7a5 5 0 0 1 10 0v4" stroke="currentColor" strokeWidth="2"/>
              </svg>
              Password
            </label>
            <input
              type="password"
              className={styles.input}
              placeholder="Enter your password"
              value={formData.password}
              onChange={(e) => setFormData({...formData, password: e.target.value})}
              required
            />
          </div>

          {error && <div className={styles.error}>{error}</div>}

          <button type="submit" className={styles.submitButton} disabled={loading}>
            {loading ? (
              <div className={styles.spinner}></div>
            ) : (
              isLogin ? 'Sign In' : 'Create Account'
            )}
          </button>
        </form>
      </div>
    </div>
  );
}
